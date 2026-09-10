# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""The cloud vision tools: read frames back out of the KVS stream.

Both invoke Go2KVSStack's SceneDescriber Lambda, which pulls frames out of the
video stream, asks Bedrock about them, and returns text for Nova Sonic to speak.

    describe_scene:      foxglove bridge ──► Bedrock              (1 hop, fast)
    kvs_describe:        KVS stream ──► Lambda ──► Bedrock        (the demo route)

`kvs_describe` is a slower copy of `describe_scene`, worth having only to show the
cloud path. `kvs_compare_scenes` is NOT a copy of `compare_scenes`, and the
difference is worth understanding:

    compare_scenes:      compares against a snapshot this session happened to take.
                         Nothing looked at that moment → nothing to compare.
    kvs_compare_scenes:  compares against the stream's 24 h archive. Works for any
                         moment in the last day, including before the agent started.

So the local one is instant but only remembers what it was asked about; the cloud one
can answer "what changed since this morning?" — which is the reason to keep a
recorded stream at all.

Requires:
  - The Go2KVSStack deployed in AWS_DEFAULT_REGION
  - Something uploading to the stream — on the cloud stack that is the container's
    kvs_producer_node; locally, launch with kvs:=true
  - Credentials with lambda:ListFunctions + lambda:InvokeFunction

Known limitation: this does NOT work from the hosted AgentCore runtime. Those ENIs
are in isolated subnets with no NAT and the VPC has no Lambda endpoint, so the
invoke cannot leave. It works from a local `make voice` run. The agent's own
describe_scene is unaffected either way, which is why it stays the default.

There used to be more here — kvs_recall, kvs_monitor_start/check/stop, and a
host-side KvsStreamer behind start_kvs_stream/stop_kvs_stream. The Lambdas behind
recall and monitor never worked (monitor ran at reserved concurrency 0 for months;
recall never produced a log line), and the streamer duplicated kvs_producer_node —
two producers interleaving fragments into one stream timeline is worse than one.
kvs_compare_scenes covers what recall was reaching for, and does it by comparison
rather than by describing a past frame in isolation.
"""

import json
import os

import boto3
from strands import tool

from . import config

_REGION = os.environ.get("AWS_DEFAULT_REGION", "us-east-1")

_lambda_client = None
_fn_name_cache: dict[str, str] = {}


def _get_lambda():
    global _lambda_client
    if _lambda_client is None:
        _lambda_client = boto3.client(
            "lambda", region_name=_REGION, config=config.boto_config()
        )
    return _lambda_client


def _find_function(prefix: str) -> str:
    """Resolve a Go2KVSStack Lambda's generated name by prefix (cached)."""
    if prefix in _fn_name_cache:
        return _fn_name_cache[prefix]

    client = _get_lambda()
    paginator = client.get_paginator("list_functions")
    for page in paginator.paginate():
        for fn in page["Functions"]:
            if fn["FunctionName"].startswith(f"Go2KVSStack-{prefix}"):
                _fn_name_cache[prefix] = fn["FunctionName"]
                return fn["FunctionName"]
    raise RuntimeError(
        f"No Lambda named Go2KVSStack-{prefix}* in {_REGION}. Is Go2KVSStack deployed?"
    )


def _invoke_lambda(prefix: str, payload: dict) -> dict:
    """Invoke a Go2KVSStack Lambda and return its parsed response."""
    fn_name = _find_function(prefix)
    response = _get_lambda().invoke(
        FunctionName=fn_name,
        Payload=json.dumps(payload).encode(),
    )
    result = json.loads(response["Payload"].read())
    if "errorMessage" in result:
        return {"status": "error", "message": result["errorMessage"]}
    return {"status": "success", "response": result.get("body", str(result))}


@tool
def kvs_describe(question: str = "") -> dict:
    """Ask the cloud to describe what the robot's camera sees, via the KVS stream.

    Uses the full cloud pipeline: a Lambda grabs the newest frame out of the
    Kinesis video stream, sends it to Bedrock, and returns the description. Slower
    than describe_scene, which reads the camera directly — use this only when the
    user explicitly asks for the cloud/KVS route.

    Args:
        question: What to ask about the scene (e.g. "what's on the table?").
                  Empty gives a general description.

    Use this when the user says things like "kvs describe scene", "what does the
    cloud see", "describe via KVS", or when demonstrating the KVS pipeline.
    """
    logger.debug("tool kvs_describe: question_len=%d", len(question))
    result = _invoke_lambda("SceneDescriber", {"question": question or "What do you see?"})
    if result["status"] == "error":
        return {
            "status": "error",
            "action": "kvs_describe",
            "note": result.get("message", "Lambda invocation failed"),
        }
    return {
        "status": "success",
        "action": "kvs_describe",
        "description": result["response"],
    }


@tool
def kvs_compare_scenes(question: str = "", seconds_ago: float = 60.0) -> dict:
    """Ask the cloud what has CHANGED, comparing recorded video against now.

    A Lambda pulls two frames out of the Kinesis video stream — one from
    `seconds_ago`, one from now — and puts both in front of Bedrock to report the
    difference.

    Prefer this over compare_scenes when the moment is further back than this
    conversation, or when nothing was observed at the time: it reads the stream's
    24-hour archive, so it does not need a snapshot to have been taken then. Use
    compare_scenes for "since a minute ago" during an active session — it's faster.

    Args:
        question: Optionally narrow it (e.g. "did anyone take the parcel?").
                  Empty asks for changes in general.
        seconds_ago: How far back to compare against (e.g. 60, 600, 3600).

    Use this when the user says things like "kvs compare scene", "compare with the
    recording", "what changed since this morning according to the video", or asks
    what changed over a span longer than the current session.
    """
    logger.debug("tool kvs_compare_scenes: question_len=%d seconds_ago=%s",
                 len(question), seconds_ago)
    result = _invoke_lambda(
        "SceneDescriber",
        {"question": question, "seconds_ago": int(seconds_ago)},
    )
    if result["status"] == "error":
        return {
            "status": "error",
            "action": "kvs_compare_scenes",
            "note": result.get("message", "Lambda invocation failed"),
        }
    return {
        "status": "success",
        "action": "kvs_compare_scenes",
        "seconds_ago": int(seconds_ago),
        "description": result["response"],
    }
