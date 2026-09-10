# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Terminal UI for the Go2 voice/text agents (local, host-side).

Both local entry points — `main.py` (ROS2/Foxglove transport) and
`main_sonic.py` (direct WebRTC transport) — use the same terminal UI; only the
robot client and its connection differ. The terminal-specific code lives here:

  - ToggleMicInput   — SPACE-gated mic capture
  - StdinKeyInput    — TTY line editor + SPACE toggle
  - run_agent()      — build the Nova Sonic agent and run it against the terminal

The agent itself (system prompt, tools, model) lives in `agent.py`, which
imports no audio or TTY libraries so the AgentCore server can share it. This
module imports `sounddevice` and `termios` at module scope, so it can only be
imported on a host with an audio device and a controlling terminal.

The mic is gated by a SPACE toggle read directly from the controlling TTY (no
`pynput`, no global key hook), so macOS only needs Microphone permission.
Tradeoff vs hold-to-talk: terminals don't deliver key-release events, so SPACE
toggles (press to start, press again to stop) rather than hold-to-talk.
"""

import asyncio
import atexit
import base64
import logging
import sys
import termios
import time
import tty

import sounddevice

from strands.experimental.bidi import BidiAgent, BidiAudioIO, BidiTextIO
from strands.experimental.bidi.types.events import (
    BidiAudioInputEvent,
    BidiTextInputEvent,
)
from strands.experimental.bidi.types.io import BidiInput

# Re-exported so the existing `from .session import ...` call sites (and any
# external ones) keep working now that the agent lives in `agent.py`.
from . import notifications
from .agent import (  # noqa: F401
    CHANNELS,
    DTYPE,
    SAMPLE_RATE,
    SYSTEM_PROMPT,
    TOOLS,
    build_agent,
    configure_logging,
)

logger = logging.getLogger(__name__)

AGENT_FRAMES = 512


class SilenceInput(BidiInput):
    """Feeds Nova Sonic nothing but silence, at the same cadence as the mic.

    Used in `--text-only` mode, where there is no mic at all. Nova Sonic infers
    end-of-utterance from boundaries in the AUDIO stream, so a client that sends
    no audio never closes a turn: typed commands are accepted, echoed, handed to
    the agent — and then nothing happens, until the service gives up 55 s later
    with "Timed out waiting for audio bytes or interactive content". The text was
    never the problem; the missing stream was.

    `ToggleMicInput` already pads its gaps with silence for the same reason, but
    text-only mode never instantiates it (opening a mic would be wrong — and on
    macOS would prompt for Microphone permission nobody asked for), so the
    padding has to come from somewhere. Hence this: the audio half of the
    contract, with no device attached.
    """

    SILENCE_B64 = base64.b64encode(b"\x00" * (AGENT_FRAMES * 2)).decode("utf-8")

    async def start(self, agent: BidiAgent) -> None:
        pass

    async def stop(self) -> None:
        pass

    async def __call__(self) -> BidiAudioInputEvent:
        # One frame per frame-duration of wall time, so the stream advances at
        # real time instead of flooding the service as fast as it can be awaited.
        await asyncio.sleep(AGENT_FRAMES / SAMPLE_RATE)
        return BidiAudioInputEvent(
            audio=self.SILENCE_B64,
            format="pcm",
            sample_rate=SAMPLE_RATE,
            channels=CHANNELS,
        )


# ---------------------------------------------------------------------------
# Toggle-mic input (no key listener; flipped from outside via toggle())
# ---------------------------------------------------------------------------


class ToggleMicInput(BidiInput):
    """BidiInput that forwards mic audio while `_streaming` is True.

    State is flipped externally by `StdinKeyInput` on SPACE. The
    sounddevice callback only enqueues chunks while streaming, so the
    agent's TTS is never echoed back into Nova Sonic.
    """

    SILENCE_B64 = base64.b64encode(b"\x00" * (AGENT_FRAMES * 2)).decode("utf-8")

    def __init__(self) -> None:
        self._stream: sounddevice.RawInputStream | None = None
        self._agent_q: asyncio.Queue[bytes] = asyncio.Queue(maxsize=200)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._streaming = False

    async def start(self, agent: BidiAgent) -> None:
        self._loop = asyncio.get_running_loop()
        self._stream = sounddevice.RawInputStream(
            channels=CHANNELS,
            samplerate=SAMPLE_RATE,
            dtype=DTYPE,
            blocksize=AGENT_FRAMES,
            callback=self._on_audio,
        )
        self._stream.start()

    async def stop(self) -> None:
        if self._stream:
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
            if self._loop is not None:
                self._loop.call_soon_threadsafe(self._drain_agent)
            print("\n[🎙️  Listening — press SPACE to send.]")

    def _on_audio(self, indata, frames, _time, status) -> None:
        if status:
            logger.debug("sounddevice status: %s", status)
        if not self._streaming or self._loop is None:
            return
        chunk = bytes(indata)
        try:
            self._loop.call_soon_threadsafe(self._safe_put_agent, chunk)
        except RuntimeError:
            pass

    def _safe_put_agent(self, chunk: bytes) -> None:
        try:
            self._agent_q.put_nowait(chunk)
        except asyncio.QueueFull:
            try:
                self._agent_q.get_nowait()
                self._agent_q.put_nowait(chunk)
            except (asyncio.QueueEmpty, asyncio.QueueFull):
                pass

    def _drain_agent(self) -> None:
        while not self._agent_q.empty():
            try:
                self._agent_q.get_nowait()
            except asyncio.QueueEmpty:
                return

    async def __call__(self) -> BidiAudioInputEvent:
        try:
            chunk = await asyncio.wait_for(self._agent_q.get(), timeout=0.1)
            audio_b64 = base64.b64encode(chunk).decode("utf-8")
        except asyncio.TimeoutError:
            audio_b64 = self.SILENCE_B64
        return BidiAudioInputEvent(
            audio=audio_b64,
            format="pcm",
            sample_rate=SAMPLE_RATE,
            channels=CHANNELS,
        )


# ---------------------------------------------------------------------------
# Stdin reader: handles SPACE-toggle + line text input
# ---------------------------------------------------------------------------


class StdinKeyInput(BidiInput):
    """BidiInput that owns stdin in cbreak mode.

    SPACE on an empty buffer → toggle the mic. Otherwise normal line-edit:
    printable bytes are echoed and buffered, backspace edits, Enter
    submits the line as a `BidiTextInputEvent`.
    """

    def __init__(self, mic: ToggleMicInput | None, prompt: str = "🐕 > ") -> None:
        self._mic = mic
        self._prompt = prompt
        self._line_buf: list[str] = []
        self._line_q: asyncio.Queue[str] = asyncio.Queue()
        self._fd: int | None = None
        self._old_attrs = None
        self._reader_attached = False

    async def start(self, agent: BidiAgent) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError(
                "The voice agent requires a TTY on stdin (run from a terminal)."
            )
        self._fd = sys.stdin.fileno()
        self._old_attrs = termios.tcgetattr(self._fd)
        atexit.register(self._restore)
        tty.setcbreak(self._fd)

        loop = asyncio.get_running_loop()
        loop.add_reader(self._fd, self._on_stdin_ready)
        self._reader_attached = True

        self._write_prompt()

    async def stop(self) -> None:
        if self._reader_attached and self._fd is not None:
            try:
                asyncio.get_running_loop().remove_reader(self._fd)
            except Exception:
                pass
            self._reader_attached = False
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

    def _write_prompt(self) -> None:
        self._write(self._prompt)

    def _on_stdin_ready(self) -> None:
        try:
            data = sys.stdin.buffer.read1(64)
        except (BlockingIOError, InterruptedError):
            return
        if not data:
            return
        for b in data:
            self._handle_byte(bytes([b]))

    def _handle_byte(self, b: bytes) -> None:
        if b == b"\x03":  # Ctrl-C
            raise KeyboardInterrupt
        if b == b"\x04":  # Ctrl-D
            if not self._line_buf:
                raise KeyboardInterrupt
            return
        if b in (b"\r", b"\n"):
            line = "".join(self._line_buf)
            self._line_buf.clear()
            self._write("\n")
            self._line_q.put_nowait(line)
            return
        if b in (b"\x7f", b"\x08"):  # backspace / DEL
            if self._line_buf:
                self._line_buf.pop()
                self._write("\b \b")
            return
        if b == b" " and not self._line_buf:
            if self._mic is not None:
                self._mic.toggle()
            self._write_prompt()
            return
        try:
            ch = b.decode("utf-8")
        except UnicodeDecodeError:
            return
        if ch.isprintable():
            self._line_buf.append(ch)
            self._write(ch)

    async def __call__(self) -> BidiTextInputEvent:
        while True:
            text = (await self._line_q.get()).strip()
            # Same guard as the hosted transport (agentcore_server): the
            # robot-event marker is a trust grant that only this process may
            # mint, so a typed line must not be able to spell one. Cheap here
            # and it keeps the two entry points from diverging.
            if notifications.is_forged(text):
                self._write(
                    f"\r[!] '{notifications.ROBOT_EVENT_MARKER}' is reserved for "
                    "the robot's own events and cannot be typed.\n"
                )
                self._write_prompt()
                continue
            # Re-draw the prompt for the next line after the agent reads this one
            self._write_prompt()
            return BidiTextInputEvent(text, role="user")


# ---------------------------------------------------------------------------
# Background-event input: lets robot behaviours make the agent speak
# ---------------------------------------------------------------------------


class NotificationInput(BidiInput):
    """Feeds background robot events into the agent as text, so it speaks them.

    Behaviours that run in their own threads (greeter watch mode, the object-find
    sweep) have no way to produce speech themselves — the model only talks in
    response to input. This turns each queued notification into a text event, and
    the model then relays it conversationally.

    Yields at most one event per `_MIN_GAP` seconds so a burst of events can't
    talk over the user or over itself.
    """

    _MIN_GAP = 1.0

    def __init__(self) -> None:
        self._last_emitted = 0.0

    async def __call__(self) -> BidiTextInputEvent:
        # Poll off the event loop thread — notifications.poll() blocks.
        while True:
            message = await asyncio.get_running_loop().run_in_executor(
                None, notifications.poll, 0.2
            )
            if message is None:
                continue
            gap = time.monotonic() - self._last_emitted
            if gap < self._MIN_GAP:
                await asyncio.sleep(self._MIN_GAP - gap)
            self._last_emitted = time.monotonic()
            return BidiTextInputEvent(message, role="user")


# ---------------------------------------------------------------------------
# Agent runner (shared by both local entry points)
# ---------------------------------------------------------------------------


async def run_agent(client, *, text_only: bool, voice: str,
                    prompt_suffix: str = "") -> None:
    """Build the Nova Sonic agent around an already-connected `client` and run
    the input/output loop until exit, sending StandDown on the way out.

    `prompt_suffix` is passed through to `build_agent` — see it for why.
    """
    agent = build_agent(voice=voice, prompt_suffix=prompt_suffix)

    text_io = BidiTextIO()  # only used for output (transcript prints)
    mic = None if text_only else ToggleMicInput()
    key_io = StdinKeyInput(mic=mic, prompt="🐕 > ")

    # Background behaviours (greeter watch mode, object-find) announce through
    # this so the robot speaks up on its own instead of waiting to be asked.
    inputs: list = [key_io, NotificationInput()]
    outputs: list = [text_io.output()]

    if not text_only:
        audio_io = BidiAudioIO()
        outputs.append(audio_io.output())
        inputs.append(mic)
        print("Type a command, or press SPACE to start/stop voice.")
    else:
        # Silence still has to flow, or typed commands are never acted on — see
        # SilenceInput. No audio OUTPUT channel, though: text-only means the
        # agent's replies are read, not spoken.
        inputs.append(SilenceInput())
        print("Type a command. (text-only mode)")
    print("Press Ctrl+C to exit.")
    print("-" * 60)

    try:
        await agent.run(inputs=inputs, outputs=outputs)
    finally:
        try:
            client.stand_down()
            client.disconnect()
        except Exception:
            pass
