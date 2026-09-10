# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Scene Describer Lambda — the cloud vision path, reading back out of KVS.

Invoked directly (lambda:InvokeFunction) by the voice agent's kvs_describe and
kvs_compare_scenes tools, which speak the returned `body`. Two modes, chosen by
whether the event carries `seconds_ago`:

    {"question": "..."}                      → describe the newest frame
    {"question": "...", "seconds_ago": 60}   → compare that moment against now

Both are cloud counterparts to tools the agent already has locally. The describe
mode is a slower copy of describe_scene, worth having only to show the KVS route.
The compare mode is not a copy: the agent's local compare_scenes can only reach a
moment it happened to snapshot during this session, while this reads the stream's
24 h archive, so it works for any moment — including before the agent was running.
"""

import logging
import os
from kvs_utils import get_latest_frame, get_frame_at_offset
from bedrock_utils import describe_image, compare_images

logger = logging.getLogger()
logger.setLevel(os.environ.get("LOG_LEVEL", "INFO"))

STREAM_NAME = os.environ["STREAM_NAME"]

# Upper bound on `seconds_ago`, matching the stream's 24 h retention. Anything
# beyond it could never resolve to a frame, and a negative offset would ask
# kvs_utils for a moment in the future.
MAX_OFFSET_SECONDS = 86400

PROMPT = (
    "Describe this scene in 2-3 concise sentences. "
    "What objects and activities do you see? "
    "Be specific about colors, positions, and quantities. "
    "Do not attempt to identify any people — just note their presence and what they are doing."
)

COMPARE_PROMPT = (
    "In 2-3 concise sentences, say what CHANGED between the earlier image and now: "
    "objects added, removed or moved, people who arrived or left, activities that "
    "started or stopped. Ignore lighting shifts and small camera-angle differences — "
    "the robot may have moved slightly. If nothing meaningful changed, say so plainly. "
    "Do not attempt to identify any people."
)

NO_LIVE_FRAME = (
    "I couldn't get a current image from the video stream. "
    "The camera may not be uploading right now."
)


def handler(event, context):
    question = event.get("question", "")
    seconds_ago = event.get("seconds_ago")

    # Metadata only. The event's `question` is the user's transcribed speech, so
    # logging the payload verbatim wrote whatever was said into CloudWatch for
    # the log group's whole retention.
    logger.info(
        "scene-describer invoked: mode=%s question_len=%d",
        "compare" if seconds_ago is not None else "describe",
        len(question),
    )

    # No caller-supplied stream name. The function exists to serve ONE robot's
    # stream, the IAM policy is scoped to exactly that stream, and accepting an
    # override only widened the input surface for a parameter nothing sets.
    stream_name = STREAM_NAME

    frame_b64 = get_latest_frame(stream_name)
    if not frame_b64:
        return {"statusCode": 200, "body": NO_LIVE_FRAME}

    # --- Describe mode ---------------------------------------------------------
    if seconds_ago is None:
        # COMPOSE, never replace. PROMPT carries the privacy control ("do not
        # attempt to identify any people"), so letting a question substitute for
        # it — which any question other than one exact literal used to do — threw
        # that control away just when it mattered most ("who is this person?").
        # bedrock_utils.SYSTEM_GUARDRAIL is the backstop that cannot be displaced
        # at all; this keeps the task-level wording too.
        prompt = (
            f"{PROMPT}\n\nAnswer this specific question: {question}"
            if question else PROMPT
        )
        description = describe_image(frame_b64, prompt)
        logger.info("described frame from %s (%d chars)", stream_name, len(description))
        return {"statusCode": 200, "body": description}

    # --- Compare mode ---------------------------------------------------------
    # Validated, not just cast: a non-numeric value raised an unhandled
    # ValueError and a negative one was passed straight through to KVS as a
    # timestamp in the future.
    try:
        offset = float(seconds_ago)
    except (TypeError, ValueError):
        return {"statusCode": 400,
                "body": "seconds_ago must be a number of seconds."}
    if not 0 <= offset <= MAX_OFFSET_SECONDS:
        return {
            "statusCode": 400,
            "body": (
                "seconds_ago must be between 0 and 86400 — the stream only "
                "keeps 24 hours of video."
            ),
        }

    past_b64 = get_frame_at_offset(stream_name, offset)
    if not past_b64:
        # Distinguish "the archive doesn't reach that far" from "no camera at all",
        # because the fix is different and the agent reads this text out loud.
        return {
            "statusCode": 200,
            "body": (
                f"I have the current view, but nothing recorded from {int(offset)} "
                "seconds ago — the robot may not have been streaming then, or that "
                "moment is past the stream's 24-hour retention."
            ),
        }

    prompt = COMPARE_PROMPT
    if question:
        prompt += f"\n\nThe user specifically asked: {question}"

    comparison = compare_images(past_b64, frame_b64, prompt)
    logger.info(
        "compared %s now vs -%ds (%d chars)", stream_name, int(offset), len(comparison)
    )
    return {"statusCode": 200, "body": comparison}
