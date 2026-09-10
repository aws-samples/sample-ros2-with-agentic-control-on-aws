# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Solution attribution for the local KVS producer script.

push_to_kvs.py is run by hand on a laptop to put a webcam feed into the demo's
Kinesis video stream, so nothing deploys it and there is no CloudFormation
mapping to read — the constants below are the operative value, not a fallback.
USER_AGENT_STRING is still honoured if the shell exports one.

deployment/lib/solution.ts is the definition these mirror; keep them in step
with it on release.
"""

import os

SOLUTION_ID = "SO0367"
SOLUTION_VERSION = "v1.0.0"

USER_AGENT_STRING = (
    os.environ.get("USER_AGENT_STRING")
    or f"AWSSOLUTION/{SOLUTION_ID}/{SOLUTION_VERSION}"
)


def boto_config(**kwargs):
    """A botocore Config carrying the solution's user-agent suffix.

    `user_agent_extra` APPENDS to the SDK's own user agent rather than replacing
    it, so the boto3/botocore/python/platform parts survive — the reporting
    depends on the whole string, not just our fragment.
    """
    from botocore.config import Config

    return Config(user_agent_extra=USER_AGENT_STRING, **kwargs)
