#!/usr/bin/env bash
# Open an SSM port-forward tunnel to the Go2 foxglove bridge.
#
#   localhost:9876  ──SSM──►  robot instance:8765
#
# One hop. The container runs with --network host, so the bridge is on the
# instance's own loopback and the instance is itself the SSM target — no load
# balancer and no jumpbox in the path. (The previous Fargate design needed both:
# SSM cannot port-forward to a Fargate task, and a task's awsvpc IP moved on
# every start.)
#
# No public endpoint and no SSH key: the instance's security group allows nothing
# inbound from the internet, so access is gated entirely by IAM — whether you can
# start an SSM session.
#
# Once running, leave this terminal open and in ANOTHER terminal:
#   Lichtblick:   Open connection -> Foxglove WebSocket -> ws://localhost:9876
#   Voice agent:  make voice-cloud
#
# The hosted agent (make voice-agentcore) does NOT use this — it runs inside the
# VPC and dials the bridge's private IP directly.
#
# Requires: AWS CLI v2 + the Session Manager plugin
#   https://docs.aws.amazon.com/systems-manager/latest/userguide/session-manager-working-with-install-plugin.html
set -euo pipefail

# Honor AWS_REGION too (the CLI does) so the stack lookup doesn't silently go
# to the wrong region if AWS_REGION is exported to something else.
REGION="${AWS_REGION:-${AWS_DEFAULT_REGION:-us-east-1}}"
# Default local port is 9876, NOT 8765: Amazon Quick.app squats 8765-8770 on
# some Macs and will answer the WebSocket with a 401, so Lichtblick "connects"
# to the wrong thing. Override with GO2_LOCAL_PORT if 9876 is taken.
LOCAL_PORT="${GO2_LOCAL_PORT:-9876}"
STACK="${GO2_STACK:-Go2Ec2Stack}"

# Fail loudly if the local port is already in use — otherwise SSM can't bind it
# and clients connect to whatever squats the port instead of the bridge.
#
# The common cause is a tunnel orphaned by `make stop-ec2`: stopping the instance
# kills the session's remote end, but session-manager-plugin keeps holding the
# local port, so the next client connects to a dead socket and hangs. Habit worth
# forming: Ctrl-C the tunnel before stopping the instance.
if lsof -nP -iTCP:"$LOCAL_PORT" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "[!] Local port $LOCAL_PORT is already in use:" >&2
  lsof -nP -iTCP:"$LOCAL_PORT" -sTCP:LISTEN 2>/dev/null | tail -n +1 | head -3 >&2
  echo "    If that is a stale tunnel from a previous session, kill it and retry." >&2
  echo "    Otherwise pick another port: GO2_LOCAL_PORT=<port> $0" >&2
  exit 1
fi

INSTANCE="${GO2_INSTANCE:-}"
if [[ -z "$INSTANCE" ]]; then
  echo "[*] Resolving the robot instance from CloudFormation stack '$STACK'..."
  # `|| INSTANCE=""` is load-bearing: under `set -e` a failed command
  # substitution aborts the script, so an undeployed stack would die on a raw
  # ValidationError instead of reaching the hint below.
  INSTANCE=$(aws cloudformation describe-stacks \
    --stack-name "$STACK" --region "$REGION" \
    --query "Stacks[0].Outputs[?OutputKey=='InstanceId'].OutputValue" \
    --output text 2>/dev/null) || INSTANCE=""
fi

if [[ -z "$INSTANCE" || "$INSTANCE" == "None" ]]; then
  echo "[!] Could not resolve an instance id. Is $STACK deployed in $REGION?" >&2
  echo "    Deploy it:         make deploy-ec2" >&2
  echo "    Override directly: GO2_INSTANCE=i-0123... $0" >&2
  exit 1
fi

echo "[*] Instance: $INSTANCE (bridge on its own localhost:8765)"
echo "[*] Tunnel:   ws://localhost:$LOCAL_PORT  ->  bridge"
echo "[*] Ctrl-C to close. Keep this terminal open while using Lichtblick/agent."

exec aws ssm start-session \
  --region "$REGION" \
  --target "$INSTANCE" \
  --document-name AWS-StartPortForwardingSession \
  --parameters "{\"portNumber\":[\"8765\"],\"localPortNumber\":[\"$LOCAL_PORT\"]}"
