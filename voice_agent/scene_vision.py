# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Scene description, robot memory and scene comparison — shared by transports.

The "what do you see?" / "what did you see earlier?" / "what changed?" tools are
identical work no matter where the frame came from: JPEG-encode it, ask a
multimodal Bedrock model, hand the text back for Nova Sonic to speak, and keep a
timestamped snapshot so the past can be asked about later.

Mixing this in means a transport only has to answer one question — "give me your
latest camera frame as a PIL image" (`_grab_live_frame`) — to get the whole
vision surface. `Go2ROS2Client` decodes it off the Foxglove bridge;
`Go2SimClient` renders it out of MuJoCo. Neither knows anything about Bedrock.
"""

from __future__ import annotations

import logging
import threading
import time

from . import config

logger = logging.getLogger(__name__)

# What the user hears when a Bedrock call fails. Deliberately not the exception
# text: botocore errors carry request IDs, ARNs and the account number, and this
# string is BOTH spoken aloud and returned into the model's context. The detail
# still goes to the log via logger.exception, where an operator can read it.
_VISION_FAILED = (
    "I couldn't get a look at that just now — my vision service didn't answer. "
    "Check the agent log for the details."
)


class SceneVisionMixin:
    """describe_scene / recall_scene / compare_scenes over any frame source.

    A host class must call `_init_scene_vision()` from its `__init__` and
    implement `_grab_live_frame(timeout)`, returning a PIL RGB image or `None`.
    """

    def _init_scene_vision(self) -> None:
        self._bedrock = None       # lazily created bedrock-runtime client
        # Robot memory: timestamped snapshots from describe_scene calls, newest
        # last. Each entry: {"ts": float, "image": PIL.Image, "description": str}.
        self._scene_memory: list = []
        self._scene_memory_lock = threading.Lock()

    # --- hooks the host may override ---------------------------------------

    def _grab_live_frame(self, timeout: float = 8.0):
        raise NotImplementedError("transports must supply a live camera frame")

    def _vision_preflight(self) -> None:
        """Raise if the transport is not in a state to produce frames.

        The ROS2 transport uses this to fail loudly on a dropped bridge instead
        of blocking for the whole frame timeout; the simulator has nothing to
        check.
        """

    # --- tools -------------------------------------------------------------

    def describe_scene(self, question: str = "", timeout: float = 8.0) -> dict:
        """Look through the robot's camera and describe what it sees.

        Grabs the latest camera frame, asks a multimodal Bedrock model to
        describe it, and returns the text so the voice agent (Nova Sonic) speaks
        it. Runs entirely on the host — no container node, no separate TTS. The
        question is forwarded so Bedrock can answer specifically (e.g. "what's on
        the table?").
        """
        self._vision_preflight()

        frame = self._grab_live_frame(timeout=timeout)
        if frame is None:
            return {"status": "error", "action": "describe_scene",
                    "note": "No camera frame received yet — make sure the camera "
                            "stream is up and try again."}

        try:
            text = self._describe_with_bedrock(frame, question)
        except Exception:  # noqa: BLE001
            logger.exception("describe_scene: Bedrock vision call failed")
            return {"status": "error", "action": "describe_scene",
                    "note": _VISION_FAILED}

        # Snapshot this observation into robot memory for later recall_scene.
        self._remember_scene(frame, text)
        return {"status": "success", "action": "describe_scene", "description": text}

    def recall_scene(self, question: str = "", seconds_ago: float = 30.0) -> dict:
        """Answer a question about what the robot saw ~seconds_ago in the past.

        Looks up the snapshot in robot memory nearest to (now - seconds_ago),
        re-queries that archived image with the question, and returns the answer
        so Nova Sonic speaks it. Memory is built by describe_scene calls during
        the session — if nothing was observed near that time, says so.
        """
        target = time.time() - max(0.0, seconds_ago)
        with self._scene_memory_lock:
            if not self._scene_memory:
                return {"status": "error", "action": "recall_scene",
                        "note": "I don't have any visual memories yet — ask me "
                                "what I see first so I start remembering."}
            # Nearest snapshot by timestamp.
            entry = min(self._scene_memory, key=lambda e: abs(e["ts"] - target))
            image = entry["image"]
            prior = entry["description"]
            actual_ago = time.time() - entry["ts"]

        prompt = (
            f"This is what your camera saw about {round(actual_ago)} seconds ago. "
            f"Answer this question about that past scene: {question}"
            if question else
            f"Describe what your camera saw about {round(actual_ago)} seconds ago."
        )
        try:
            text = self._describe_with_bedrock(image, prompt)
        except Exception:  # noqa: BLE001
            logger.exception("recall_scene: Bedrock vision call failed")
            return {"status": "error", "action": "recall_scene",
                    "note": _VISION_FAILED}
        return {
            "status": "success",
            "action": "recall_scene",
            "seconds_ago": round(actual_ago),
            "description": text,
            "prior_description": prior,
            "snapshot_path": entry.get("path"),
        }

    def compare_scenes(self, question: str = "", seconds_ago: float = 30.0,
                       timeout: float = 8.0) -> dict:
        """Compare what the robot sees now against a snapshot from the past.

        Sends BOTH the past snapshot (nearest to now - seconds_ago) and a live
        camera frame to Bedrock in one call, asking what changed between them.
        Use for "what's different?", "what changed since earlier?", "did anything
        move?". Returns the answer so Nova Sonic speaks it.
        """
        # Past frame from memory.
        target = time.time() - max(0.0, seconds_ago)
        with self._scene_memory_lock:
            if not self._scene_memory:
                return {"status": "error", "action": "compare_scenes",
                        "note": "I don't have any earlier views to compare against "
                                "yet — ask me what I see first."}
            entry = min(self._scene_memory, key=lambda e: abs(e["ts"] - target))
            past_image = entry["image"]
            actual_ago = time.time() - entry["ts"]

        # Live frame now.
        live = self._grab_live_frame(timeout=timeout)
        if live is None:
            return {"status": "error", "action": "compare_scenes",
                    "note": "No live camera frame available to compare — make sure "
                            "the camera stream is up."}

        ask = question or "What has changed?"
        prompt = (
            f"The first image is what your camera saw about {round(actual_ago)} "
            f"seconds ago. The second image is what you see right now. Compare "
            f"them and answer in one or two short, friendly spoken sentences: {ask}"
        )
        try:
            text = self._bedrock_vision([past_image, live], prompt)
        except Exception:  # noqa: BLE001
            logger.exception("compare_scenes: Bedrock vision call failed")
            return {"status": "error", "action": "compare_scenes",
                    "note": _VISION_FAILED}

        # Snapshot the live frame so it becomes part of memory too.
        self._remember_scene(live, f"(comparison vs {round(actual_ago)}s ago) {text}")
        return {
            "status": "success",
            "action": "compare_scenes",
            "seconds_ago": round(actual_ago),
            "description": text,
        }

    # --- memory ------------------------------------------------------------

    def _remember_scene(self, pil_image, description: str) -> None:
        """Store a timestamped (image, description) snapshot, capped at MAX."""
        entry = {"ts": time.time(), "image": pil_image, "description": description}
        with self._scene_memory_lock:
            self._scene_memory.append(entry)
            if len(self._scene_memory) > config.SCENE_MEMORY_MAX:
                self._scene_memory.pop(0)
        if config.SCENE_MEMORY_DIR:
            self._persist_snapshot(entry)

    def _persist_snapshot(self, entry: dict) -> None:
        """Optionally write the snapshot JPEG to disk (for the monitor to show).

        These frames are camera imagery of whoever is in front of the robot, so
        the directory is created 0700 and swept after every write — see
        `_prune_snapshots`. Off unless SCENE_MEMORY_DIR is set.
        """
        import os

        try:
            os.makedirs(config.SCENE_MEMORY_DIR, mode=0o700, exist_ok=True)
            path = os.path.join(config.SCENE_MEMORY_DIR, f"scene_{int(entry['ts'])}.jpg")
            entry["image"].save(path, format="JPEG", quality=85)
            os.chmod(path, 0o600)
            entry["path"] = path
        except Exception:  # noqa: BLE001
            logger.exception("snapshot persist failed")
        else:
            self._prune_snapshots()

    @staticmethod
    def _prune_snapshots() -> None:
        """Enforce the on-disk snapshot TTL and count cap.

        Without this the directory grows without limit for as long as the agent
        runs, holding pictures of everyone who walked past. Both bounds are
        applied on every write rather than on a timer, so there is no thread to
        leak and no window where a crashed sweeper leaves the cap unenforced.

        Deletion is best-effort per file: a frame the monitor still has open is
        not a reason to abandon the rest of the sweep.
        """
        import os

        directory = config.SCENE_MEMORY_DIR
        cutoff = time.time() - config.SCENE_MEMORY_TTL_SECONDS
        try:
            names = [n for n in os.listdir(directory)
                     if n.startswith("scene_") and n.endswith(".jpg")]
        except OSError:
            logger.debug("snapshot prune: %s is not listable", directory)
            return

        # Newest first, so the count cap keeps the most recent frames. The
        # filename carries the timestamp, so no stat() is needed to order them.
        names.sort(reverse=True)
        for index, name in enumerate(names):
            path = os.path.join(directory, name)
            try:
                too_old = os.path.getmtime(path) < cutoff
            except OSError:
                continue
            if too_old or index >= config.SCENE_MEMORY_DISK_MAX:
                try:
                    os.remove(path)
                except OSError:
                    logger.debug("snapshot prune: could not remove %s", name)

    # --- Bedrock -----------------------------------------------------------

    def _describe_with_bedrock(self, pil_image, question: str) -> str:
        """Single-image convenience wrapper over _bedrock_vision."""
        return self._bedrock_vision([pil_image], question or config.SCENE_DEFAULT_PROMPT)

    def _bedrock_vision(self, pil_images: list, prompt: str) -> str:
        """Ask Bedrock (Converse API) about one or more JPEG-encoded images.

        Images appear in the message in list order, each preceded by no caption;
        the prompt text (which can reference "the first image" / "the second
        image") comes last. Used for both single-frame description and two-frame
        comparison.
        """
        import io

        import boto3

        if self._bedrock is None:
            session = boto3.Session(profile_name=config.AWS_BEDROCK_PROFILE)
            self._bedrock = session.client("bedrock-runtime",
                                           region_name=config.SCENE_REGION,
                                           config=config.boto_config())

        content = []
        for img in pil_images:
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=85)
            content.append({"image": {"format": "jpeg",
                                      "source": {"bytes": buf.getvalue()}}})
        content.append({"text": prompt})

        # guardrail_config() is empty when no guardrail is deployed, which keeps
        # a credentials-only local run working; when one IS configured this is
        # what applies PROMPT_ATTACK screening to the image's own text and PII
        # anonymisation to the description before it is spoken.
        resp = self._bedrock.converse(
            modelId=config.SCENE_MODEL_ID,
            messages=[{"role": "user", "content": content}],
            system=[{"text": config.SCENE_SYSTEM_PROMPT}],
            inferenceConfig={"maxTokens": config.SCENE_MAX_TOKENS},
            **config.guardrail_config(),
        )
        # A guardrail intervention returns a stop reason and a canned message
        # rather than model output. Say so plainly instead of speaking the
        # blocked-message boilerplate as if it were a description.
        if resp.get("stopReason") == "guardrail_intervened":
            logger.warning("Bedrock guardrail blocked a vision response")
            return ("I looked, but I'm not able to describe that one — my "
                    "safety filter stopped the answer.")
        return resp["output"]["message"]["content"][0]["text"].strip()
