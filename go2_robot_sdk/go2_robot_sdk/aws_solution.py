# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
"""Solution attribution for the AWS SDK calls this ROS 2 package makes.

Every boto3 client built in this package appends `AWSSOLUTION/<id>/<version>` to
the User-Agent header, which is how the solution's service API usage is reported.
The callers are the KVS producer node and the WebRTC connection's Secrets
Manager fetch.

On the cloud path the string arrives as the USER_AGENT_STRING environment
variable: Go2Ec2Stack writes it into /etc/go2.env from its `Solution`
CloudFormation mapping and go2-run.sh passes it into the container. That mapping
is the single definition — see deployment/lib/solution.ts.

The constants below are the fallback for the paths CloudFormation is not on: a
docker-compose run on a Mac, or a bare `ros2 launch` against the robot over the
LAN. Keep them in step with solution.ts on release.
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

    botocore is imported lazily so importing this module stays free for the ROS
    nodes that never touch AWS.
    """
    from botocore.config import Config

    return Config(user_agent_extra=USER_AGENT_STRING, **kwargs)
