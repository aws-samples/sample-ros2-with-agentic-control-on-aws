# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Push Mac webcam to KVS using opencv + ffmpeg + boto3 PutMedia.

Strategy: capture N seconds to a temp MKV file, then upload to KVS.
This two-phase approach avoids pipe buffering issues.
"""

import os
import sys
import time
import subprocess
import tempfile
import signal
import datetime
import json

import cv2
import boto3
import requests
from botocore.auth import SigV4Auth
from botocore.awsrequest import AWSRequest
from botocore.session import Session

from aws_solution import USER_AGENT_STRING, boto_config


STREAM_NAME = os.environ.get("KVS_STREAM_NAME", "go2-robot-01-camera")
REGION = os.environ.get("AWS_DEFAULT_REGION", "us-east-1")
FPS = 15
WIDTH = 1280
HEIGHT = 720
DURATION_SECONDS = int(os.environ.get("DURATION", "30"))

running = True


def signal_handler(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def get_put_media_endpoint(stream_name: str) -> str:
    client = boto3.client("kinesisvideo", region_name=REGION, config=boto_config())
    response = client.get_data_endpoint(
        StreamName=stream_name,
        APIName="PUT_MEDIA",
    )
    return response["DataEndpoint"]


def capture_to_mkv(output_path: str) -> int:
    """Capture webcam frames via ffmpeg to an MKV file. Returns frame count."""
    global running

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    if not cap.isOpened():
        print("ERROR: Could not open webcam")
        sys.exit(1)

    # The argv list is written out here rather than built into a variable first, so
    # the program name stays visibly a literal. argv-list form with no shell=True:
    # no shell is spawned, so the non-constant elements (frame geometry, fps and
    # output_path, all computed locally) cannot be reinterpreted as shell syntax.
    proc = subprocess.Popen(
        [
            "ffmpeg", "-y", "-loglevel", "error",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{WIDTH}x{HEIGHT}",
            "-r", str(FPS),
            "-i", "pipe:0",
            "-pix_fmt", "yuv420p",
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-tune", "zerolatency",
            "-profile:v", "baseline",
            "-level", "4.0",
            "-g", str(FPS),
            "-bf", "0",
            "-b:v", "2000k",
            "-f", "matroska",
            "-cluster_time_limit", "500",
            output_path,
        ],
        stdin=subprocess.PIPE, stderr=subprocess.PIPE,
    )

    frame_count = 0
    start_time = time.time()

    while running:
        elapsed = time.time() - start_time
        if elapsed >= DURATION_SECONDS:
            break

        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        try:
            proc.stdin.write(frame.tobytes())
        except (BrokenPipeError, OSError):
            break

        frame_count += 1
        if frame_count % (FPS * 5) == 0:
            print(f"  Captured {frame_count} frames ({elapsed:.0f}s)")

        # Throttle to FPS
        expected = frame_count / FPS
        if expected > elapsed:
            time.sleep(expected - elapsed)

    proc.stdin.close()
    proc.wait()
    cap.release()
    return frame_count


def upload_mkv_to_kvs(mkv_path: str, endpoint: str):
    """Upload an MKV file to KVS via PutMedia."""
    url = f"{endpoint}/putMedia"

    session = Session()
    credentials = session.get_credentials().get_frozen_credentials()

    now = datetime.datetime.now(datetime.timezone.utc)
    start_timestamp = now.strftime("%Y-%m-%dT%H:%M:%S.000Z")

    headers = {
        "x-amzn-stream-name": STREAM_NAME,
        "x-amzn-fragment-timecode-type": "RELATIVE",
        "x-amzn-producer-start-timestamp": start_timestamp,
        "Content-Type": "application/json",
    }

    request = AWSRequest(method="POST", url=url, headers=headers, data=b"")
    SigV4Auth(credentials, "kinesisvideo", REGION).add_auth(request)

    signed_headers = dict(request.headers)
    # PutMedia is a raw streaming POST, not a boto3 call, so the solution string
    # goes on by hand. Set after signing: botocore keeps user-agent out of SigV4
    # (SIGNED_HEADERS_BLACKLIST), so this cannot invalidate the signature.
    signed_headers["User-Agent"] = (
        f"{requests.utils.default_user_agent()} {USER_AGENT_STRING}"
    )
    signed_headers["x-amzn-stream-name"] = STREAM_NAME
    signed_headers["x-amzn-fragment-timecode-type"] = "RELATIVE"
    signed_headers["x-amzn-producer-start-timestamp"] = start_timestamp
    signed_headers["Transfer-Encoding"] = "chunked"

    file_size = os.path.getsize(mkv_path)
    print(f"  Uploading {file_size / 1024:.0f} KB to KVS...")

    def file_chunks():
        with open(mkv_path, "rb") as f:
            while True:
                chunk = f.read(16384)
                if not chunk:
                    break
                yield chunk

    response = requests.post(
        url,
        headers=signed_headers,
        data=file_chunks(),
        stream=True,
        timeout=30,
    )

    print(f"  PutMedia status: {response.status_code}")

    persisted = 0
    errors = 0
    for line in response.iter_lines():
        if line:
            try:
                ack = json.loads(line)
                event_type = ack.get("EventType", "")
                if event_type == "PERSISTED":
                    persisted += 1
                    frag = ack.get("FragmentNumber", "")[:20]
                    print(f"  ✓ Fragment {persisted} persisted ({frag}...)")
                elif event_type == "ERROR":
                    errors += 1
                    print(f"  ✗ {ack.get('ErrorCode')}: {ack.get('ErrorMessage', '')}")
            except json.JSONDecodeError:
                pass

    print(f"  Upload complete: {persisted} fragments persisted, {errors} errors")
    return persisted > 0


def main():
    print(f"=== KVS Webcam Test ===")
    print(f"Stream: {STREAM_NAME} | Region: {REGION}")
    print(f"Capture: {WIDTH}x{HEIGHT} @ {FPS}fps for {DURATION_SECONDS}s\n")

    # Phase 1: Capture
    print(f"[Phase 1] Capturing {DURATION_SECONDS}s of webcam video...")
    # mkstemp rather than a fixed /tmp name: the file is created atomically with
    # 0600, so a second run (or another user on the box) can't race or read it.
    # ffmpeg is called with -y, so writing over the empty placeholder is fine.
    fd, mkv_path = tempfile.mkstemp(prefix="kvs_webcam_capture_", suffix=".mkv")
    os.close(fd)
    frame_count = capture_to_mkv(mkv_path)
    print(f"  Done: {frame_count} frames → {mkv_path}")

    if frame_count == 0:
        print("ERROR: No frames captured")
        sys.exit(1)

    # Phase 2: Upload
    print(f"\n[Phase 2] Uploading to KVS...")
    endpoint = get_put_media_endpoint(STREAM_NAME)
    success = upload_mkv_to_kvs(mkv_path, endpoint)

    if success:
        print(f"\n✓ Video is now in KVS stream '{STREAM_NAME}'")
        print(f"  You can verify in the AWS console: KVS → Video streams → {STREAM_NAME} → Media playback")
    else:
        print("\n✗ Upload may have failed — check KVS console")

    # Cleanup
    os.remove(mkv_path)


if __name__ == "__main__":
    main()
