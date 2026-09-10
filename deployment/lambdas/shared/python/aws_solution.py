# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Solution attribution for the AWS SDK calls these Lambdas make.

Every boto3 client built here appends `AWSSOLUTION/<id>/<version>` to the
User-Agent header, which is how this solution's service API usage is reported.

The string comes from the USER_AGENT_STRING environment variable, which
Go2KVSStack sets from its `Solution` CloudFormation mapping — that mapping is
the single definition (see deployment/lib/solution.ts). The constants below
are only a fallback for running this code outside CloudFormation, e.g. invoking
the handler locally; keep them in step with solution.ts on release.
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
