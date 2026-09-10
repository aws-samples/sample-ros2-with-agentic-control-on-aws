#!/usr/bin/env bash
# Control the Go2 robot instance deployed by Go2Ec2Stack.
#
#   ./scripts/ec2-ctl.sh start     boot the instance; go2.service comes up with it
#   ./scripts/ec2-ctl.sh stop      stop it — compute billing stops, EBS (~$8/mo) stays
#   ./scripts/ec2-ctl.sh restart   re-pull the image and restart the container in place
#   ./scripts/ec2-ctl.sh status    instance state + container state + recent unit log
#   ./scripts/ec2-ctl.sh mode <remote|bridge>   switch who owns the robot link
#   ./scripts/ec2-ctl.sh open      opt-in: allow this machine straight to :8765
#                                  (needs GO2_ACK_UNAUTHENTICATED_BRIDGE=yes)
#   ./scripts/ec2-ctl.sh close     revoke every rule `open` created
#   ./scripts/ec2-ctl.sh host      print the instance's public IP (for `make voice-ec2`)
#   ./scripts/ec2-ctl.sh extend [min]            push the auto-stop out (default 60)
#   ./scripts/ec2-ctl.sh autostop <min|off|on|default>  change/disarm the budget
#
# One script rather than six because every verb needs the same instance id, and
# that id is also the SSM tunnel target scripts/tunnel.sh uses. One resource plays
# every role here, which is the whole reason there is no load balancer and no
# jumpbox to address separately.
#
# ## open / close
#
# `make tunnel` (SSM port-forward) is THE supported way in. It needs no open port
# and access is gated by IAM. `open` is an escape hatch, not a step: it adds a
# security-group rule letting THIS machine's public address reach 8765 directly,
# which is faster for camera and pointcloud topics because it does not relay every
# byte through the SSM service.
#
# It refuses to run unless you set GO2_ACK_UNAUTHENTICATED_BRIDGE=yes, because
# what it publishes to the internet is an UNAUTHENTICATED ROS 2 control plane —
# arbitrary topic publish, /cmd_vel and sport commands included. Source-IP gating
# is the only control, and it is a weak one: behind an office NAT, a café, or
# CGNAT, "this machine's address" is everyone sharing that egress. Use it on a lab
# robot in a cleared space or not at all.
#
# Three things bound it once you have opted in:
#
#   - /32 only. The detected address is validated and pinned to a single host;
#     this never writes a range and can never write 0.0.0.0/0.
#   - Scoped cleanup. `close` revokes BY RULE ID every ingress rule on 8765 whose
#     CIDR is not the VPC's — including rules added by hand, which carry no
#     description and would otherwise linger forever. The CDK-managed in-VPC rule
#     is excluded by comparison and cannot be caught.
#   - Auto-revoked. `stop` closes before stopping, and `start` closes first too,
#     since a leftover rule aimed at a stale address is pure exposure with no
#     upside.
#
# These rules live outside CloudFormation on purpose (see the long comment in
# deployment/lib/go2-ec2-stack.ts). A `cdk deploy` will also wipe them, which
# is the desired failure mode, not a bug.
#
# ## The auto-stop, from this side
#
# A watchdog Lambda in Go2Ec2Stack stops the instance once it has been running for
# longer than its uptime budget (4h by default), because at ~$0.526/hr a forgotten
# `stop` is the most expensive thing that can happen here. Every knob is an
# instance TAG, so `extend` and `autostop` are one API call, not a deploy:
#
#   go2:autostop-minutes  the budget itself, counted from the last START
#   go2:autostop-until    an absolute UTC reprieve; `extend` writes this
#   go2:autostop=off      disarmed — then nothing will ever stop it but you
#
# `status` prints the budget and the time remaining, and `start` clears any stale
# reprieve so a session always begins with the full budget and nothing inherited.
#
# `restart` is the routine way to roll a new image: it pulls over cached layers
# and restarts the unit, so it costs seconds and needs no stack update. Bumping
# the stack's ImageTag parameter would REPLACE the instance and throw the layer
# cache away.
#
# Requires: AWS CLI v2 (+ the Session Manager plugin for tunnel.sh, not here).
set -euo pipefail

ACTION="${1:-status}"

# Honor AWS_REGION too (the CLI does) so a stack lookup can't silently go to the
# wrong region when AWS_REGION is exported to something else.
REGION="${AWS_REGION:-${AWS_DEFAULT_REGION:-us-east-1}}"
STACK="${GO2_EC2_STACK:-Go2Ec2Stack}"

INSTANCE="${GO2_INSTANCE:-}"
if [[ -z "$INSTANCE" ]]; then
  # To stderr, not stdout: `host` is consumed by a command substitution in the
  # Makefile, and a progress line on stdout would end up inside the hostname.
  echo "[*] Resolving instance id from CloudFormation stack '$STACK'..." >&2
  # `|| INSTANCE=""` is load-bearing: under `set -e` a failed command
  # substitution aborts the script, so the un-deployed case would die on a raw
  # ValidationError instead of reaching the hint below.
  INSTANCE=$(aws cloudformation describe-stacks \
    --stack-name "$STACK" --region "$REGION" \
    --query "Stacks[0].Outputs[?OutputKey=='InstanceId'].OutputValue" \
    --output text 2>/dev/null) || INSTANCE=""
fi

if [[ -z "$INSTANCE" || "$INSTANCE" == "None" ]]; then
  echo "[!] Could not resolve an instance id. Is $STACK deployed in $REGION?" >&2
  echo "    Deploy it:         make deploy-ec2" >&2
  echo "    Override directly: GO2_INSTANCE=i-0123... $0 $ACTION" >&2
  exit 1
fi

# Stamped on every rule this script creates, purely so `describe-security-group-rules`
# and CloudTrail show where a rule came from. It is NOT how `close` selects rules —
# see external_bridge_rule_ids() for why that would have missed hand-added ones.
MARKER="go2-ctl-temp-direct"
BRIDGE_PORT=8765

state() {
  aws ec2 describe-instances --instance-ids "$INSTANCE" --region "$REGION" \
    --query 'Reservations[0].Instances[0].State.Name' --output text
}

instance_field() {
  aws ec2 describe-instances --instance-ids "$INSTANCE" --region "$REGION" \
    --query "Reservations[0].Instances[0].$1" --output text
}

sg_id() { instance_field 'SecurityGroups[0].GroupId'; }
vpc_cidr() {
  aws ec2 describe-vpcs --region "$REGION" \
    --vpc-ids "$(instance_field VpcId)" \
    --query 'Vpcs[0].CidrBlock' --output text
}

# Every INGRESS rule on the bridge port whose CIDR is not the VPC's — i.e.
# everything reachable from outside the VPC.
#
# Deliberately not "rules matching $MARKER". Matching only our own tag looked
# tidier but failed the actual requirement: a rule added by hand from the console
# or the CLI carries no description, so it would survive `stop` indefinitely —
# precisely the lingering exposure this is meant to prevent. Anything on 8765 that
# is not the one CDK-managed in-VPC rule is treated as temporary and removed.
#
# The CDK rule (the VPC CIDR) is excluded by comparison, and rules that reference
# a security group rather than a CIDR have no CidrIpv4 and are skipped, so neither
# can be caught by accident.
external_bridge_rule_ids() {
  local cidr
  cidr=$(vpc_cidr)
  aws ec2 describe-security-group-rules --region "$REGION" \
    --filters "Name=group-id,Values=$(sg_id)" \
    --query "SecurityGroupRules[?IsEgress==\`false\` && FromPort==\`$BRIDGE_PORT\` && CidrIpv4!=null && CidrIpv4!='$cidr'].SecurityGroupRuleId" \
    --output text 2>/dev/null || true
}

# Idempotent: safe to call when nothing is open.
close_direct() {
  local ids
  ids=$(external_bridge_rule_ids)
  if [[ -z "$ids" || "$ids" == "None" ]]; then
    echo "[*] No external access rules on :$BRIDGE_PORT to revoke."
    return 0
  fi
  echo "[*] Revoking external access rule(s) on :$BRIDGE_PORT: $ids"
  # By rule id, so there is no way to catch a neighbouring rule that happens to
  # share a port or CIDR.
  aws ec2 revoke-security-group-ingress --region "$REGION" \
    --group-id "$(sg_id)" --security-group-rule-ids $ids >/dev/null
  echo "[*] Closed. Direct access is off; 'make tunnel' still works."
}

# --- Auto-stop -------------------------------------------------------------
# Names must match the constants in deployment/lib/go2-ec2-stack.ts; the Lambda
# there is the only reader.
ARMED_TAG="go2:autostop"
BUDGET_TAG="go2:autostop-minutes"
UNTIL_TAG="go2:autostop-until"

# LaunchTime plus all three tags in ONE describe call, tab-separated, "None" for
# whatever is unset. LaunchTime is the last START for an EBS-backed instance, which
# is what the budget is counted from.
autostop_fields() {
  aws ec2 describe-instances --instance-ids "$INSTANCE" --region "$REGION" \
    --query "Reservations[0].Instances[0].[LaunchTime, Tags[?Key=='$BUDGET_TAG']|[0].Value, Tags[?Key=='$UNTIL_TAG']|[0].Value, Tags[?Key=='$ARMED_TAG']|[0].Value]" \
    --output text
}

tag_set() {
  aws ec2 create-tags --resources "$INSTANCE" --region "$REGION" \
    --tags "Key=$1,Value=$2" >/dev/null
}

tag_clear() {
  aws ec2 delete-tags --resources "$INSTANCE" --region "$REGION" \
    --tags "Key=$1" >/dev/null 2>&1 || true
}

# The deploy-time default, read from the stack output rather than repeated here, so
# this script cannot disagree with the constant the Lambda actually uses. Only the
# fallback is hardcoded, for a stack deployed before the output existed.
stack_budget() {
  local v
  v=$(aws cloudformation describe-stacks --stack-name "$STACK" --region "$REGION" \
    --query "Stacks[0].Outputs[?OutputKey=='AutoStopMinutes'].OutputValue" \
    --output text 2>/dev/null) || v=""
  [[ "$v" =~ ^[0-9]+$ ]] && echo "$v" || echo 240
}

# One line about when this instance stops itself. Best-effort throughout: this is a
# report, and no failure in it should take a verb down with it.
autostop_report() {
  local launch budget reprieve armed
  # 'reprieve', not 'until': until is a bash reserved word and there is no reason
  # to find out the hard way where that does and does not matter.
  IFS=$'\t' read -r launch budget reprieve armed < <(autostop_fields) || return 0
  [[ "$armed" == "off" ]] && {
    echo "[!] auto-stop is DISARMED ($ARMED_TAG=off) — only you can stop this instance."
    return 0
  }
  [[ "$budget" =~ ^[0-9]+$ ]] || budget=$(stack_budget)
  [[ "$reprieve" == "None" ]] && reprieve=""
  # python3 rather than date(1): BSD date on macOS cannot parse an ISO-8601 offset,
  # and the arithmetic has to agree exactly with the Lambda's.
  command -v python3 >/dev/null || return 0
  python3 -c '
import datetime, sys

launch, budget, until = sys.argv[1], int(sys.argv[2]), sys.argv[3]
now = datetime.datetime.now(datetime.timezone.utc)
started = datetime.datetime.fromisoformat(launch.replace("Z", "+00:00"))
deadline = started + datetime.timedelta(minutes=budget)
note = ""
if until:
    try:
        asked = datetime.datetime.fromisoformat(until.replace("Z", "+00:00"))
        if asked.tzinfo is None:
            asked = asked.replace(tzinfo=datetime.timezone.utc)
        if asked > deadline:          # extensions only ever push it out
            deadline, note = asked, " (extended)"
    except ValueError:
        pass
left = (deadline - now).total_seconds() / 60.0
print("[*] auto-stop%s: %.0f min left of a %d min budget (up %.0f min, stops %s local)"
      % (note, max(left, 0), budget, (now - started).total_seconds() / 60.0,
         deadline.astimezone().strftime("%H:%M")))
if left <= 15:
    print("    That is soon. Push it out:  make extend-ec2        (+60 min)")
' "$launch" "$budget" "$reprieve" 2>/dev/null || true
}

case "$ACTION" in
  start)
    # A rule left over from a previous session points at an address that may no
    # longer be yours, so it is exposure buying nothing. Clear before booting.
    close_direct
    # Likewise for a reprieve granted to the PREVIOUS session: the watchdog would
    # ignore a past timestamp anyway, but an hour-old extension still showing up in
    # `status` reads as though it applies to this session.
    tag_clear "$UNTIL_TAG"
    echo "[*] Starting $INSTANCE ($REGION)..."
    aws ec2 start-instances --instance-ids "$INSTANCE" --region "$REGION" \
      --query 'StartingInstances[0].{instance:InstanceId,from:PreviousState.Name,to:CurrentState.Name}' \
      --output table
    echo "[*] Waiting for the instance to reach 'running'..."
    aws ec2 wait instance-running --instance-ids "$INSTANCE" --region "$REGION"
    # The instance is 'running' well before the ROS graph is up: systemd still
    # has to start docker, pull (cached), and wait on the TURN handshake.
    echo "[*] Instance running. The robot stack needs ~30-40s more (cached image)."
    echo "    Watch it:   aws logs tail /go2/ec2 --follow --region $REGION"
    echo "    Then:       make tunnel      (SSM, IAM-gated, no open port)"
    # The budget is counted from this start, so say out loud when the box goes away
    # on its own. Nobody should learn about the auto-stop from a dead session.
    autostop_report
    ;;

  stop)
    # Before stopping, not after: this is the guarantee that a session cannot
    # leave the robot reachable from the internet.
    close_direct
    echo "[*] Stopping $INSTANCE ($REGION)..."
    # systemd runs go2.service's ExecStop on shutdown, so the container gets a
    # clean SIGTERM and the WebRTC session closes properly rather than timing
    # out on the robot's side.
    aws ec2 stop-instances --instance-ids "$INSTANCE" --region "$REGION" \
      --query 'StoppingInstances[0].{instance:InstanceId,from:PreviousState.Name,to:CurrentState.Name}' \
      --output table
    echo "[*] Stopping. Compute billing ends at 'stopped'; the EBS volume (and its"
    echo "    cached image layers) persists so the next start stays fast."
    ;;

  restart)
    now=$(state)
    if [[ "$now" != "running" ]]; then
      echo "[!] Instance is '$now', not running — use '$0 start' first." >&2
      exit 1
    fi
    echo "[*] Re-pulling the image and restarting go2.service on $INSTANCE..."
    cmd=$(aws ssm send-command \
      --instance-ids "$INSTANCE" --region "$REGION" \
      --document-name AWS-RunShellScript \
      --parameters 'commands=["systemctl restart go2.service"]' \
      --query 'Command.CommandId' --output text)
    echo "[*] SSM command $cmd sent; waiting..."
    # The unit's ExecStart does the docker pull, so this returns as soon as the
    # process is up — not when the robot has connected.
    aws ssm wait command-executed \
      --command-id "$cmd" --instance-id "$INSTANCE" --region "$REGION" || true
    aws ssm get-command-invocation \
      --command-id "$cmd" --instance-id "$INSTANCE" --region "$REGION" \
      --query '{status:Status,stdout:StandardOutputContent,stderr:StandardErrorContent}' \
      --output table
    echo "[*] Restarted. Existing tunnels survive (same host, same port) but the"
    echo "    bridge dropped, so reconnect Lichtblick / the agent."
    ;;

  status)
    now=$(state)
    echo "[*] Instance $INSTANCE ($REGION): $now"
    if [[ "$now" == "running" ]]; then
      autostop_report
      # Each command is `|| true` so one missing piece (unit not up yet, docker
      # still starting) still lets the rest of the report through.
      cmd=$(aws ssm send-command \
        --instance-ids "$INSTANCE" --region "$REGION" \
        --document-name AWS-RunShellScript \
        --parameters 'commands=["echo \"robot link mode: $(cat /etc/go2.mode 2>/dev/null || echo remote)\"","if docker exec go2 env 2>/dev/null | grep -q ^UNITREE_; then echo \"unitree creds in container: YES (can reach Unitree cloud)\"; else echo \"unitree creds in container: no (cannot reach Unitree cloud)\"; fi","systemctl is-active go2.service || true","docker ps --filter name=go2 --format \"{{.Status}}\" || true","journalctl -u go2 -n 15 --no-pager || true"]' \
        --query 'Command.CommandId' --output text)
      aws ssm wait command-executed \
        --command-id "$cmd" --instance-id "$INSTANCE" --region "$REGION" || true
      aws ssm get-command-invocation \
        --command-id "$cmd" --instance-id "$INSTANCE" --region "$REGION" \
        --query 'StandardOutputContent' --output text
    else
      echo "    Nothing to report from the box while it is '$now'. 'make start-ec2' first."
    fi
    ;;

  mode)
    # Switch which end owns the robot link, WITHOUT replacing the instance.
    #
    #   remote  this instance dials the dog through Unitree's TURN server
    #   bridge  a laptop on the dog's own AP owns the link and feeds this ROS
    #           graph (scripts/go2_ap_bridge.py); this instance makes NO calls to
    #           Unitree at all, because go2-run.sh skips the credential fetch
    #           entirely in this mode
    #
    # The mode lives in /etc/go2.mode (not /run, which is tmpfs and would revert
    # on reboot) and is read by go2-run.sh at every unit start, so applying it is
    # just a container restart — seconds, against the minutes and cold 14 GB pull
    # an instance replacement would cost.
    WANT="${2:-}"
    case "$WANT" in
      remote|bridge) ;;
      "") echo "[!] usage: $0 mode <remote|bridge>" >&2; exit 1 ;;
      *)  echo "[!] unknown mode '$WANT' (expected remote or bridge)" >&2; exit 1 ;;
    esac
    if [[ "$(state)" != "running" ]]; then
      echo "[!] instance is not running; 'make start-ec2' first" >&2
      exit 1
    fi
    echo "[*] setting robot link mode to '$WANT' and restarting the container..."
    if [[ "$WANT" == bridge ]]; then
      echo "    In bridge mode this instance holds NO robot link and never contacts"
      echo "    Unitree. Start the laptop side with 'make ap-bridge'."
    fi
    cmd=$(aws ssm send-command \
      --instance-ids "$INSTANCE" --region "$REGION" \
      --document-name AWS-RunShellScript \
      --parameters "commands=[\"echo $WANT >/etc/go2.mode\",\"systemctl restart go2.service\",\"sleep 20\",\"echo mode=\$(cat /etc/go2.mode)\",\"echo unit=\$(systemctl is-active go2.service)\",\"if ! systemctl is-active --quiet go2.service; then echo; echo '=== the unit did NOT come up - last 20 log lines ==='; journalctl -u go2 -n 20 --no-pager; fi\"]" \
      --query 'Command.CommandId' --output text) || {
        echo "[!] send-command failed" >&2; exit 1; }
    aws ssm wait command-executed \
      --command-id "$cmd" --instance-id "$INSTANCE" --region "$REGION" || true
    aws ssm get-command-invocation \
      --command-id "$cmd" --instance-id "$INSTANCE" --region "$REGION" \
      --query 'StandardOutputContent' --output text
    echo "[*] 'make status-ec2' to confirm; the ROS graph takes ~30s to come back."
    if [[ "$WANT" == bridge ]]; then
      echo "[i] If the log above shows a ROS parameter error about a tuple, or"
      echo "    \"Robot IPs: []\", the running IMAGE predates bridge support."
      echo "    Fix: make push-image && make restart-ec2 (the stack deploy does not"
      echo "    rebuild the image)."
    fi
    ;;

  open)
    # Explicit opt-in, checked before anything else. This is the one path in the
    # repo that puts an unauthenticated actuator on a public address, and a
    # documented `make open-ec2` reads like a normal step unless it stops you.
    if [[ "${GO2_ACK_UNAUTHENTICATED_BRIDGE:-}" != "yes" ]]; then
      cat >&2 <<'EOF'
[!] REFUSING: 'open' publishes an UNAUTHENTICATED ROS 2 control plane
    (tcp/8765) to a public address. Anyone who reaches it can drive the robot —
    arbitrary topic publish, /cmd_vel and sport commands included. The only
    control is your source IP, which is shared on office NAT, café Wi-Fi
    and CGNAT.

    Use the supported path instead. It needs no ingress rule and is gated by IAM:
        make tunnel        # SSM port forwarding, scripts/tunnel.sh

    If you accept the risk on a lab robot in a cleared space:
        GO2_ACK_UNAUTHENTICATED_BRIDGE=yes make open-ec2

    Close it as soon as you are done: make close-ec2
EOF
      exit 1
    fi

    now=$(state)
    if [[ "$now" != "running" ]]; then
      echo "[!] Instance is '$now'. Start it first: make start-ec2" >&2
      exit 1
    fi

    # An explicit override wins, otherwise ask AWS what address it sees — that is
    # the one that has to appear in the rule, not whatever a local interface
    # thinks it is. Resolve first, THEN validate, so the override is checked too.
    MY_IP="${GO2_MY_IP:-}"
    if [[ -z "$MY_IP" ]]; then
      MY_IP=$(curl -fsS --max-time 10 https://checkip.amazonaws.com | tr -d '[:space:]') || MY_IP=""
    fi
    # Bare IPv4 only. Rejecting anything with a prefix length is what makes it
    # impossible for this path to produce a range, /0 included.
    if [[ ! "$MY_IP" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
      echo "[!] Not a usable public IPv4: '${MY_IP:-nothing}'." >&2
      echo "    Refusing to guess — a wrong CIDR is either a dead connection or an" >&2
      echo "    over-broad rule. Pass a bare address: GO2_MY_IP=<a.b.c.d> $0 open" >&2
      exit 1
    fi
    CIDR="$MY_IP/32"

    # Replace rather than accumulate: re-running after your address changes should
    # not leave the old one authorized.
    close_direct

    echo "[*] Allowing $CIDR -> $(sg_id) tcp/$BRIDGE_PORT ..."
    aws ec2 authorize-security-group-ingress --region "$REGION" \
      --group-id "$(sg_id)" \
      --ip-permissions "IpProtocol=tcp,FromPort=$BRIDGE_PORT,ToPort=$BRIDGE_PORT,IpRanges=[{CidrIp=$CIDR,Description=$MARKER}]" \
      >/dev/null

    PUB=$(instance_field PublicIpAddress)
    echo
    echo "    Lichtblick / agent:  ws://$PUB:$BRIDGE_PORT"
    echo "    Voice agent:         make voice-ec2      (resolves this address itself)"
    echo
    echo "[!] The bridge is UNAUTHENTICATED and now reachable from $CIDR."
    echo "    'make close-ec2' when you're done — 'make stop-ec2' also revokes it."
    ;;

  close)
    close_direct
    ;;

  extend)
    # Add N minutes to the CURRENT deadline and write the result, as an absolute
    # timestamp, into $UNTIL_TAG.
    #
    # Absolute rather than "+N minutes to the budget" because it then expires by
    # itself: a tag left over from an earlier session is simply a moment in the
    # past, which the watchdog ignores (it only ever takes a reprieve that is LATER
    # than the budget deadline), so there is no stale state to clean up anywhere.
    #
    # Relative to the deadline rather than to now, because "now + 60" would be a
    # silent no-op when run with more than an hour still left — you would ask for
    # an extension, be told you got one, and get nothing. Running it twice adds
    # twice, which is what "extend" should mean.
    MINUTES="${2:-60}"
    if [[ ! "$MINUTES" =~ ^[0-9]+$ ]] || (( MINUTES == 0 )); then
      echo "[!] usage: $0 extend [minutes]   (a positive whole number, default 60)" >&2
      exit 1
    fi
    if [[ "$(state)" != "running" ]]; then
      echo "[!] Instance is not running; there is nothing to extend." >&2
      exit 1
    fi
    IFS=$'\t' read -r launch budget reprieve armed < <(autostop_fields)
    [[ "$budget" =~ ^[0-9]+$ ]] || budget=$(stack_budget)
    [[ "$reprieve" == "None" ]] && reprieve=""
    UNTIL=$(python3 -c '
import datetime, sys

launch, budget, reprieve, add = sys.argv[1], int(sys.argv[2]), sys.argv[3], int(sys.argv[4])
now = datetime.datetime.now(datetime.timezone.utc)
deadline = datetime.datetime.fromisoformat(launch.replace("Z", "+00:00")) \
    + datetime.timedelta(minutes=budget)
if reprieve:
    try:
        asked = datetime.datetime.fromisoformat(reprieve.replace("Z", "+00:00"))
        if asked.tzinfo is None:
            asked = asked.replace(tzinfo=datetime.timezone.utc)
        deadline = max(deadline, asked)
    except ValueError:
        pass
# From now, if the deadline has already passed and the watchdog has not got to it
# yet: extending should always buy the full N minutes.
print((max(deadline, now) + datetime.timedelta(minutes=add)).strftime("%Y-%m-%dT%H:%M:%SZ"))
' "$launch" "$budget" "$reprieve" "$MINUTES")
    tag_set "$UNTIL_TAG" "$UNTIL"
    echo "[*] Auto-stop pushed out by $MINUTES min, to $UNTIL."
    # The tag is read on the watchdog's next 5-minute poll, so an extension applied
    # in the last few minutes of the budget is still in time — the stop and the
    # reprieve are read in the same invocation.
    autostop_report
    ;;

  autostop)
    # Change the budget itself, or disarm. Unlike `extend`, this PERSISTS across
    # stop/start, because it is a statement about this instance rather than about
    # one session.
    WANT="${2:-}"
    case "$WANT" in
      off)
        tag_set "$ARMED_TAG" off
        echo "[!] Auto-stop DISARMED. Nothing will stop this instance now except you,"
        echo "    at ~\$0.526/hr — and an unattended robot dialing Unitree's cloud is"
        echo "    also how the source IP ends up WAF-blocked. Re-arm: $0 autostop on"
        ;;
      on)
        tag_clear "$ARMED_TAG"
        echo "[*] Auto-stop re-armed."
        autostop_report
        ;;
      default)
        # Drop all three tags, so the budget goes back to whatever the stack was
        # deployed with. Without this there is no way to undo an `autostop 600`
        # except knowing the deploy-time number by heart.
        tag_clear "$ARMED_TAG"
        tag_clear "$BUDGET_TAG"
        tag_clear "$UNTIL_TAG"
        echo "[*] Back to the stack default ($(stack_budget) min), no extension."
        autostop_report
        ;;
      "")
        echo "[!] usage: $0 autostop <minutes|off|on|default>" >&2
        echo "    current setting:" >&2
        autostop_report >&2
        exit 1
        ;;
      *)
        if [[ ! "$WANT" =~ ^[0-9]+$ ]] || (( WANT == 0 )); then
          echo "[!] '$WANT' is not a budget. Give whole minutes, or off/on/default." >&2
          exit 1
        fi
        tag_set "$BUDGET_TAG" "$WANT"
        # Re-arming too: setting a budget while disarmed and having nothing happen
        # would be the more surprising behaviour.
        tag_clear "$ARMED_TAG"
        echo "[*] Uptime budget is now $WANT min, counted from each start."
        autostop_report
        ;;
    esac
    ;;

  # Just the public IP on stdout, so a caller can build ws://<ip>:8765 without
  # parsing anything. Everything else this prints goes to stderr.
  #
  # It also refuses to hand back an address that cannot be reached: the instance
  # must be running AND an external rule must exist on the bridge port. Without
  # that check the agent would just hang on connect (a dropped SYN, not a refusal)
  # for the full websocket timeout and then report a bridge problem — when the
  # actual fix is one `make open-ec2`.
  host)
    now=$(state)
    if [[ "$now" != "running" ]]; then
      echo "[!] Instance is '$now'. Start it first: make start-ec2" >&2
      exit 1
    fi
    ids=$(external_bridge_rule_ids)
    if [[ -z "$ids" || "$ids" == "None" ]]; then
      echo "[!] Nothing outside the VPC may reach :$BRIDGE_PORT — the instance is" >&2
      echo "    running but the port is closed, so a direct connection would hang." >&2
      echo "    Go through SSM instead:  make tunnel + make voice-cloud" >&2
      echo "    (Or accept the unauthenticated exposure for this machine only:" >&2
      echo "     GO2_ACK_UNAUTHENTICATED_BRIDGE=yes make open-ec2)" >&2
      exit 1
    fi
    PUB=$(instance_field PublicIpAddress)
    if [[ -z "$PUB" || "$PUB" == "None" ]]; then
      echo "[!] Instance $INSTANCE has no public IP address." >&2
      exit 1
    fi
    echo "$PUB"
    ;;

  *)
    echo "usage: $0 {start|stop|restart|status|mode|open|close|host|extend|autostop}" >&2
    exit 2
    ;;
esac
