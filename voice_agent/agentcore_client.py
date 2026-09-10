# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Thin local client for the Go2 agent hosted on Bedrock AgentCore Runtime.

The agent, its tools, and the robot link all live in the cloud
(`agentcore_server.py`); the microphone and speaker are inherently on your
laptop, so this client does nothing but move audio:

    mic ──16 kHz PCM──► wss://bedrock-agentcore.../runtimes/<arn>/ws ──► agent
    speaker ◄──────────────────── agent audio ◄──────────────────────────┘

The terminal UI matches the local agent (`voice_agent.main`): SPACE on an empty
prompt toggles the mic, typed lines are sent as text, Ctrl+C exits. What's gone
is everything else — no Strands, no boto3 Bedrock calls, no robot client. It
needs only `sounddevice`, `websockets`, and `bedrock-agentcore` (for SigV4).

Usage — the runtime ARN is read from the Go2AgentCoreStack outputs, so there is
nothing to copy and paste. Your terminal's AWS profile and region are used as-is:

    python -m voice_agent.agentcore_client
    python -m voice_agent.agentcore_client --text-only
    python -m voice_agent.agentcore_client --url ws://localhost:8080/ws   # local server

Overrides, in precedence order: `--agent-arn`, `$AGENT_ARN`, then the stack
lookup (`--stack` to name a different stack).

Requires `bedrock-agentcore:InvokeAgentRuntimeWithWebSocketStream` on your
caller identity, plus `cloudformation:DescribeStacks` for the ARN lookup.
"""

import argparse
import asyncio
import atexit
import base64
import json
import os
import sys
import termios
import tty

import sounddevice
import websockets

# Nova Sonic's audio contract — must match the server's `agent.SAMPLE_RATE`.
SAMPLE_RATE = 16000
CHANNELS = 1
DTYPE = "int16"
FRAMES = 512  # 512 samples @ 16 kHz = 32 ms per frame

# Nova Sonic needs a continuous audio stream — it derives end-of-utterance from
# trailing silence. When the mic is off (or between callbacks) we send a frame of
# silence instead of letting the stream stall. Matches ToggleMicInput in
# session.py, which does the same for the local agent.
SILENCE_FRAME = b"\x00" * (FRAMES * 2)  # int16 mono
IDLE_TIMEOUT = 0.1  # seconds to wait for real audio before sending silence

# AgentCore gives each session its own microVM, and it is only provisioned once
# the connection arrives — so the HTTP 101 waits on a cold start (measured at
# ~12 s for this image; longer if the image isn't cached). `websockets` defaults
# `open_timeout` to 10 s, which is under that: the client gives up, the microVM
# finishes booting into a socket nobody is holding, and the runtime log shows a
# healthy "session started" for a session the user was told had timed out.
OPEN_TIMEOUT = 120

# Audio-only deps here on purpose: this module must import on a machine with no
# AWS agent stack installed. `bedrock_agentcore` is imported lazily in
# `_resolve_url` so `--url` (local testing) works without it.


DEFAULT_STACK = "Go2AgentCoreStack"
ARN_OUTPUT_KEY = "AgentRuntimeArn"


def _arn_from_stack(stack_name: str) -> str:
    """Read the runtime ARN from the CloudFormation stack's outputs.

    Uses the terminal's ambient AWS profile and region (boto3's default
    credential/region chain) — no flags, nothing to paste. Raises SystemExit
    with an actionable message if the stack or output isn't there.
    """
    import boto3
    from botocore.exceptions import BotoCoreError, ClientError, NoRegionError

    from . import config

    try:
        cfn = boto3.client("cloudformation", config=config.boto_config())
        region = cfn.meta.region_name
        print(f"[*] Looking up {ARN_OUTPUT_KEY} from {stack_name} in {region} ...", flush=True)
        outputs = cfn.describe_stacks(StackName=stack_name)["Stacks"][0].get("Outputs", [])
    except NoRegionError:
        raise SystemExit(
            "[!] No AWS region configured. Set AWS_REGION (or AWS_DEFAULT_REGION), "
            "or pass --agent-arn."
        )
    except (ClientError, BotoCoreError) as e:
        raise SystemExit(
            f"[!] Could not read stack {stack_name!r}: {e}\n"
            "    Check your AWS profile/region point at the account holding the\n"
            "    stack (aws sts get-caller-identity), or pass --agent-arn directly."
        )

    for out in outputs:
        if out.get("OutputKey") == ARN_OUTPUT_KEY:
            return out["OutputValue"]

    raise SystemExit(
        f"[!] Stack {stack_name!r} has no {ARN_OUTPUT_KEY} output. Is the agent "
        "deployed? (make deploy-agentcore)"
    )


def _resolve_url(agent_arn: str) -> str:
    """Build a SigV4 pre-signed wss:// URL for the deployed runtime.

    Pre-signed (credentials in the query string) rather than signed headers so
    the URL is all `websockets.connect` needs. It expires in 5 minutes, which
    only has to outlast the handshake — the session itself continues after.

    Signs with the terminal's ambient region via the boto3 session, so run this
    from a shell whose AWS_REGION matches the runtime's. A mismatch fails the
    handshake with a bare HTTP 403 (see the error text in `run()`).
    """
    import boto3
    from bedrock_agentcore.runtime import AgentCoreRuntimeClient

    client = AgentCoreRuntimeClient(session=boto3.Session())
    return client.generate_presigned_url(runtime_arn=agent_arn, expires=300)


class Speaker:
    """Plays agent PCM audio, with a buffer that can be dropped on barge-in."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._stream: sounddevice.RawOutputStream | None = None

    def start(self) -> None:
        self._stream = sounddevice.RawOutputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype=DTYPE,
            blocksize=FRAMES,
            callback=self._on_output,
        )
        self._stream.start()

    def stop(self) -> None:
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None

    def play(self, pcm: bytes) -> None:
        self._buf.extend(pcm)

    def clear(self) -> None:
        """Drop buffered audio so an interrupted response stops immediately."""
        self._buf.clear()

    def _on_output(self, outdata, frames, _time, status) -> None:
        wanted = frames * 2  # int16 mono
        chunk = bytes(self._buf[:wanted])
        del self._buf[:wanted]
        if len(chunk) < wanted:
            chunk += b"\x00" * (wanted - len(chunk))  # pad with silence
        outdata[:] = chunk


class Microphone:
    """SPACE-gated mic capture; enqueues PCM only while streaming."""

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=200)
        self._stream: sounddevice.RawInputStream | None = None
        self._streaming = False

    def start(self) -> None:
        self._stream = sounddevice.RawInputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype=DTYPE,
            blocksize=FRAMES,
            callback=self._on_audio,
        )
        self._stream.start()

    def stop(self) -> None:
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None

    def toggle(self) -> None:
        if self._streaming:
            self._streaming = False
            print("\n[ ] Mic off.")
        else:
            self._streaming = True
            self._drain()
            print("\n[🎙️  Listening — press SPACE to send.]")

    def _drain(self) -> None:
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except asyncio.QueueEmpty:
                return

    def _on_audio(self, indata, frames, _time, status) -> None:
        if not self._streaming:
            return
        chunk = bytes(indata)
        try:
            self._loop.call_soon_threadsafe(self._put, chunk)
        except RuntimeError:
            pass

    def _put(self, chunk: bytes) -> None:
        try:
            self._queue.put_nowait(chunk)
        except asyncio.QueueFull:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(chunk)
            except (asyncio.QueueEmpty, asyncio.QueueFull):
                pass

    async def get(self) -> bytes:
        """Return the next mic chunk, or a frame of silence if the mic is idle.

        Nova Sonic is a continuous-audio model: it infers end-of-utterance from
        trailing silence. If the stream simply stops when the mic is off, the
        model never sees that boundary and holds your turn open — so nothing
        happens until the NEXT time you speak, which then supplies the silence
        that closes the previous utterance. That is the one-turn lag and the
        run-together transcripts ("stand up.crab").

        Returning silence on a short timeout keeps the stream flowing, exactly
        like the local agent's ToggleMicInput does.
        """
        try:
            return await asyncio.wait_for(self._queue.get(), timeout=IDLE_TIMEOUT)
        except asyncio.TimeoutError:
            return SILENCE_FRAME


class Keyboard:
    """TTY line editor: SPACE on an empty buffer toggles the mic, Enter sends.

    Same behaviour as the local agent's `StdinKeyInput` — cbreak mode read
    straight off the controlling terminal, so it must run from a real TTY.
    """

    def __init__(self, mic: Microphone | None, prompt: str = "🐕 > ") -> None:
        self._mic = mic
        self._prompt = prompt
        self._buf: list[str] = []
        self._lines: asyncio.Queue[str] = asyncio.Queue()
        self._fd: int | None = None
        self._old_attrs = None
        self._attached = False

    def start(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("This client requires a TTY on stdin (run from a terminal).")
        self._fd = sys.stdin.fileno()
        self._old_attrs = termios.tcgetattr(self._fd)
        atexit.register(self._restore)
        tty.setcbreak(self._fd)
        asyncio.get_running_loop().add_reader(self._fd, self._on_ready)
        self._attached = True
        self._write(self._prompt)

    def stop(self) -> None:
        if self._attached and self._fd is not None:
            try:
                asyncio.get_running_loop().remove_reader(self._fd)
            except Exception:
                pass
            self._attached = False
        self._restore()

    def _restore(self) -> None:
        if self._old_attrs is not None and self._fd is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)
            except Exception:
                pass
            self._old_attrs = None

    def _write(self, s: str) -> None:
        sys.stdout.write(s)
        sys.stdout.flush()

    def _on_ready(self) -> None:
        try:
            data = sys.stdin.buffer.read1(64)
        except (BlockingIOError, InterruptedError):
            return
        for b in data or b"":
            self._handle(bytes([b]))

    def _handle(self, b: bytes) -> None:
        if b == b"\x03":  # Ctrl-C
            raise KeyboardInterrupt
        if b == b"\x04":  # Ctrl-D on an empty line
            if not self._buf:
                raise KeyboardInterrupt
            return
        if b in (b"\r", b"\n"):
            line = "".join(self._buf)
            self._buf.clear()
            self._write("\n")
            self._lines.put_nowait(line)
            return
        if b in (b"\x7f", b"\x08"):  # backspace / DEL
            if self._buf:
                self._buf.pop()
                self._write("\b \b")
            return
        if b == b" " and not self._buf:
            if self._mic is not None:
                self._mic.toggle()
            self._write(self._prompt)
            return
        try:
            ch = b.decode("utf-8")
        except UnicodeDecodeError:
            return
        if ch.isprintable():
            self._buf.append(ch)
            self._write(ch)

    async def get_line(self) -> str:
        line = await self._lines.get()
        self._write(self._prompt)
        return line


async def _pump_mic(ws, mic: Microphone) -> None:
    while True:
        chunk = await mic.get()
        await ws.send(
            json.dumps({"type": "audio", "audio": base64.b64encode(chunk).decode()})
        )


async def _pump_silence(ws) -> None:
    """Keep the audio stream alive in text-only mode (no mic to pace it).

    Nova Sonic answers on audio boundaries, so a text-only session with no audio
    at all gets a reply queued but never spoken/finalized. Feeding silence at
    real-time pace gives the model the cadence it expects, which is what makes
    typed commands actually take effect.
    """
    frame = base64.b64encode(SILENCE_FRAME).decode()
    interval = FRAMES / SAMPLE_RATE  # 32 ms — real-time pacing
    while True:
        await asyncio.sleep(interval)
        await ws.send(json.dumps({"type": "audio", "audio": frame}))


async def _pump_keyboard(ws, keyboard: Keyboard) -> None:
    while True:
        line = await keyboard.get_line()
        if line.strip():
            await ws.send(json.dumps({"type": "text", "text": line.strip()}))


async def _pump_server(ws, speaker: Speaker | None) -> None:
    """Handle server frames: play audio, print transcripts and tool calls."""
    last_role = None
    async for raw in ws:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            continue

        kind = msg.get("type")
        if kind == "audio":
            if speaker is not None:
                speaker.play(base64.b64decode(msg["audio"]))

        elif kind == "transcript":
            role, text = msg.get("role"), msg.get("text", "")
            if not text:
                continue
            if role != last_role:
                print(f"\n{'🤖' if role == 'assistant' else '🗣️ '} ", end="", flush=True)
                last_role = role
            print(text, end="", flush=True)

        elif kind == "tool":
            print(f"\n[tool] {msg.get('name')}", flush=True)

        elif kind == "interruption":
            if speaker is not None:
                speaker.clear()

        elif kind == "error":
            print(f"\n[!] {msg.get('message')}", flush=True)

        elif kind == "ready":
            print("[✓] Agent ready.\n", flush=True)


async def run(url: str, text_only: bool) -> None:
    loop = asyncio.get_running_loop()

    mic = None if text_only else Microphone(loop)
    speaker = None if text_only else Speaker()
    keyboard = Keyboard(mic=mic)

    print(f"[*] Connecting to {url.split('?')[0]} ...", flush=True)
    print("    (first connect cold-starts the runtime — can take ~15 s)", flush=True)
    try:
        ws = await websockets.connect(url, max_size=None, open_timeout=OPEN_TIMEOUT)
    except Exception as e:
        # Report handshake failures in plain language. A raw traceback here is
        # actively misleading: the useful line is buried, and the most common
        # causes are environmental, not bugs.
        status = getattr(getattr(e, "response", None), "status_code", None)
        print(f"\n[!] Could not connect: {type(e).__name__}", file=sys.stderr)
        if status == 403:
            print(
                "    HTTP 403 — the handshake was rejected. Usually one of:\n"
                "      * wrong AWS account/credentials for this runtime\n"
                "        (check: aws sts get-caller-identity)\n"
                "      * signing region != the ARN's region (a stale AWS_REGION\n"
                "        in your shell overrides AWS_DEFAULT_REGION)\n"
                "      * missing bedrock-agentcore:InvokeAgentRuntimeWithWebSocketStream",
                file=sys.stderr,
            )
        elif status == 404:
            print(
                "    HTTP 404 — no such runtime. Check the ARN and that the stack\n"
                "    is deployed (aws bedrock-agentcore-control list-agent-runtimes).",
                file=sys.stderr,
            )
        elif isinstance(e, TimeoutError):
            print(
                f"    No handshake response within {OPEN_TIMEOUT}s. The runtime\n"
                "    accepts the connection only after its microVM boots, so check\n"
                "    whether the container is crashing on startup:\n"
                "      aws logs tail /aws/bedrock-agentcore/runtimes/<runtime-id>-DEFAULT",
                file=sys.stderr,
            )
        else:
            print(f"    {e}", file=sys.stderr)
        raise SystemExit(1)

    # `ws` is already an open connection (awaited above so handshake errors are
    # reported cleanly), so close it via the connection's own context manager.
    async with ws:
        print("[✓] Connected.")
        if mic is not None and speaker is not None:
            mic.start()
            speaker.start()
            print("Type a command, or press SPACE to start/stop voice.")
        else:
            print("Type a command. (text-only mode)")
        print("Press Ctrl+C to exit.")
        print("-" * 60)

        keyboard.start()
        tasks = [
            asyncio.create_task(_pump_server(ws, speaker)),
            asyncio.create_task(_pump_keyboard(ws, keyboard)),
        ]
        if mic is not None:
            tasks.append(asyncio.create_task(_pump_mic(ws, mic)))
        else:
            # No mic to pace the stream, so generate the silence ourselves —
            # without it, typed commands are received but never acted on.
            tasks.append(asyncio.create_task(_pump_silence(ws)))

        try:
            # First task to finish ends the session — normally _pump_server
            # when the server closes the socket.
            await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:
                task.cancel()
            keyboard.stop()
            if mic is not None:
                mic.stop()
            if speaker is not None:
                speaker.stop()
            try:
                await ws.send(json.dumps({"type": "close"}))
            except Exception:
                pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Thin client for the Go2 agent on AgentCore Runtime"
    )
    parser.add_argument("--text-only", action="store_true", help="Disable mic and playback")
    parser.add_argument(
        "--url",
        default=None,
        help="Connect to this WebSocket URL directly (e.g. ws://localhost:8080/ws "
        "for a locally-run server). Skips SigV4 signing.",
    )
    parser.add_argument(
        "--agent-arn",
        default=os.environ.get("AGENT_ARN"),
        help="AgentCore Runtime ARN. Default: read from the stack's "
        f"{ARN_OUTPUT_KEY} output (or $AGENT_ARN if set).",
    )
    parser.add_argument(
        "--stack",
        default=os.environ.get("GO2_AGENTCORE_STACK", DEFAULT_STACK),
        help=f"Stack to read the ARN from (default: {DEFAULT_STACK}, "
        "or $GO2_AGENTCORE_STACK)",
    )
    args = parser.parse_args()

    if args.url:
        url = args.url
    else:
        url = _resolve_url(args.agent_arn or _arn_from_stack(args.stack))

    try:
        asyncio.run(run(url, args.text_only))
    except KeyboardInterrupt:
        print("\n[*] Bye!")


if __name__ == "__main__":
    main()
