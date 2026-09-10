# Go2 ROS2 SDK — AWS Deployment Guide

> ⚠️ **Sample code — not for production use.** This repository is a demonstration
> sample with no AWS SLA or support. Read the
> [non-production disclaimer](README.md#%EF%B8%8F-sample-code--not-for-production-use)
> and [RESPONSIBLE-AI.md](RESPONSIBLE-AI.md) before adapting any of it.

Deploying and operating the cloud side: the CDK stacks, the robot instance, the
hosted voice agent, and the day-to-day `make` targets that drive them.

> **Just want to run the demo?** The [README](README.md) covers it end-to-end.
> This guide is for **operators** deploying or maintaining the stacks.

**Looking for the local setup?** Running the SDK on macOS with Docker, SLAM and
Nav2, object detection, and the full command reference are in
[DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md).

## Contents

- [Architecture](#architecture)
- [One-time setup (operators)](#one-time-setup-operators)
- [Daily use](#daily-use)
- [Gotchas](#gotchas)
- [Debugging a robot that won't connect](#debugging-a-robot-that-wont-connect)

---

## Architecture

Normally the dog and the SDK container have to be on the same Wi-Fi network so
WebRTC can reach the robot over the LAN. To run the container **in the cloud**
instead, it connects to the robot through Unitree's TURN server
(`CONN_TYPE=remote`) — no shared network required. The robot just needs internet.

Four stacks, one VPC:

| Stack | What it is |
|---|---|
| [`Go2KVSStack`](deployment/lib/go2-kvs-stack.ts) | KVS stream + scene-analysis Lambdas + IoT rules. Serverless, no VPC. |
| [`Go2NetworkStack`](deployment/lib/go2-network-stack.ts) | The shared VPC. Public subnets (robot), isolated subnets (agent ENIs), and the interface endpoints those ENIs reach AWS through. |
| [`Go2Ec2Stack`](deployment/lib/go2-ec2-stack.ts) | The robot: one EC2 instance running the SDK container at a fixed private IP. |
| [`Go2AgentCoreStack`](deployment/lib/go2-agentcore-stack.ts) | The hosted voice agent, in the same VPC, dialling that private IP. |

```
┌────────── your Mac ──────────┐   ┌──────────── Go2NetworkStack VPC ────────────┐
│  Lichtblick ─┐               │   │  public 10.0.1.0/24                          │
│  voice-cloud ─┴▶ :9876 ──SSM─┼──▶│    robot instance 10.0.1.10:8765 ──┐         │
└──────────────────────────────┘   │    (no inbound from the internet)   │  TURN   │
                                   │  private 10.0.2.0/24               │         │
   voice-agentcore ──wss──▶ AgentCore Runtime ──┘ (no tunnel)            │         │
                                   └────────────────────────────────────┼─────────┘
                                                                 ┌──────▼──────┐
                                                                 │  Go2 robot  │
                                                                 └─────────────┘
```

**Why one instance and not Fargate.** This used to run on ECS Fargate, which
needed managed egress, an internal NLB, and an SSM jumpbox — about **$57/month of
always-on infrastructure** whether or not a robot was running — and re-pulled the
14 GB image on every single start (2–4 minutes), because Fargate keeps no layer
cache. Every one of those pieces existed to work around two properties of a
Fargate task: it has no host, and no stable address. An instance is both, so all
three went away. What's left costs **~$8/month stopped** — plus the VPC endpoints
below — and starts in ~40s.

The instance is a **`g4dn.xlarge`** (4 vCPU + one NVIDIA T4, ~$0.526/hr running).
The GPU is there for the object detector, which is the bottleneck in the whole
system: on CPU the accurate backbone took ~3.9 s/frame and was unusable, and even
the default took ~380 ms. The T4 removes that budget, so the cloud stack runs the
highest-accuracy backbone (`coco_model:=best`, 46.7 mAP) instead of the 37.0 mAP
compromise. The hourly rate only applies while you're actually driving — about 44
cents/hour more than the CPU instance it replaced.

The one thing Fargate was genuinely better at was running several robots at once
(`desiredCount: N`). If that ever comes back, so does a scheduler.

**The four interface endpoints (~$29/month) are for the hosted agent, not the
robot.** The robot egresses through the IGW from its public subnet for free.
AgentCore can't do that: its ENIs never get a public IP, so a public subnet gives it
a `0.0.0.0/0` route that silently blackholes (HTTP 424, empty log group). It lives
in `PRIVATE_ISOLATED` subnets and reaches exactly four services over PrivateLink —
`ecr.api` + `ecr.dkr` + the free `s3` gateway endpoint for the image pull, `logs`
for CloudWatch and X-Ray, and `bedrock-runtime` for Nova Sonic. All four are
load-bearing; each was proven by watching the runtime fail without it.

They bill **per AZ** ($0.01/hr ≈ $7.30/mo each), so they are pinned to the robot's
AZ only — spreading four endpoints across both subnets would double the bill.
Private DNS still resolves for the ENI in the other AZ, which just pays pennies of
cross-AZ transfer on a cold image pull.

Two consequences worth knowing:

- **KVS has no PrivateLink endpoint in `us-east-1`**, so the agent's `kvs_*` tools
  can't call it directly. Upload still works because `kvs_producer_node` runs on the
  instance and uses its IGW egress.
- **A warm runtime hides missing egress.** An already-provisioned AgentCore version
  keeps working after you remove an endpoint, because its image is cached on the
  host; it breaks on the next cold provision, possibly hours later. Force a new
  runtime version before concluding an endpoint is unnecessary.

If you drop `Go2AgentCoreStack`, delete the interface endpoints in
[go2-network-stack.ts](deployment/lib/go2-network-stack.ts) and the whole VPC
becomes free.

**Security posture.** The robot's security group allows exactly one thing inbound:
TCP 8765 from inside the VPC, which is how the hosted agent reaches the bridge.
From a laptop there is no inbound path by default — you tunnel over SSM, so access
is gated by IAM (who can start a session), not by an open port. **The SSM tunnel is
the supported way in, and it is the only one this guide recommends.**

The bridge itself is unauthenticated — not partially, not by obscurity: anyone who
reaches tcp/8765 can publish arbitrary ROS 2 topics, `/cmd_vel` and sport commands
included, and drive the robot. The network is the whole control, which is why the
security group admits only the VPC CIDR.

`make open-ec2` is an escape hatch that breaks that property: it punches a
temporary /32 hole for video-rate work (see Daily use), leaving the robot's control
plane reachable from a public address, gated only by source IP. It now refuses to
run unless you set `GO2_ACK_UNAUTHENTICATED_BRIDGE=yes`. The rule lives outside
CloudFormation and is revoked by `close-ec2`, `stop-ec2`, `start-ec2`, the auto-stop
watchdog, and any `cdk deploy`.

## One-time setup (operators)

```bash
# 1. Store the whole robot link in one secret. None of it belongs in a
#    CloudFormation parameter — those are plaintext to anyone who can call
#    describe-stacks. The stacks read all three keys at container start, which is
#    also why the deploys below take no arguments.
aws secretsmanager create-secret --name unitree_go2_account_password \
  --secret-string '{"email":"you@example.com","password":"YOUR_UNITREE_PASSWORD","serial":"B42D2000XXXXXXXX"}'
make check-secret          # verifies all three keys, prints names only

# 2. Push the image FIRST, or the instance boots into a crash-loop on a missing
#    one. This target creates the ECR repo if it doesn't exist yet.
make cdk-install
npx cdk bootstrap          # first time only, per account/region (from deployment)
make push-image

# 3. Deploy, in order. deploy-cloud does all three; the individual targets exist
#    because each is `--exclusively` and won't reach sideways into its siblings.
make deploy-cloud          # = deploy-network, deploy-ec2, deploy-agentcore
make status-ec2            # instance + container state + the unit log
```

> `--platform linux/amd64` is baked into `push-image` and matters on Apple
> Silicon. You also need the [Session Manager plugin](https://docs.aws.amazon.com/systems-manager/latest/userguide/session-manager-working-with-install-plugin.html)
> for the tunnel.

### The KVS stack

`deploy-cloud` covers the VPC, the robot instance and the hosted agent.
`Go2KVSStack` — the KVS stream and the `SceneDescriber` Lambda behind the
`kvs_*` voice commands — is deployed from [`deployment/`](deployment/)
directly, once per AWS account/region (Docker must be running — the Lambda is a
container image):

```bash
cd deployment
# --frozen-lockfile, not a bare install: it installs exactly what
# pnpm-lock.yaml records and fails rather than silently resolving something else.
pnpm install --frozen-lockfile
npx cdk deploy
```

See [`deployment/README.md`](deployment/README.md) for full setup
instructions, and
[KVS Camera Streaming](DEVELOPER_GUIDE.md#kvs-camera-streaming) for what the
stream is used for and how to enable the producer in a local container.

## Daily use

```bash
make start-ec2         # boot (~40s to a live bridge — the image is cached)
make tunnel            # terminal A — ws://localhost:9876, leave open
make voice-cloud       # terminal B — laptop-side agent
                       #   or Lichtblick → Foxglove WebSocket → ws://localhost:9876
make stop-ec2          # done; ~$8/mo of EBS is all that remains (GPU billing ends)
```

**If you forget `stop-ec2`, the instance stops itself after 4 hours.** A watchdog
Lambda in `Go2Ec2Stack` polls every 5 minutes and stops the instance once it has
been running past its uptime budget, counted from the last *start*. A forgotten
instance is the only thing here that costs real money (~$12.60/day), and it's also
a robot dialing Unitree's cloud unattended — which is how the source IP ends up
WAF-blocked. `make status-ec2` prints the countdown, and `make start-ec2` reports
when the session will end.

Both knobs are instance **tags**, so changing either is one API call, not a deploy:

```bash
make extend-ec2                    # +60 min on this session (MINUTES=90 for more)
make autostop-ec2 MINUTES=480      # a bigger budget, persists across stop/start
make autostop-ec2 MINUTES=off      # disarm entirely — then only you can stop it
make autostop-ec2 MINUTES=default  # back to the stack's 4 hours
```

`extend-ec2` adds to the current deadline (so running it twice adds twice) and
writes an absolute timestamp, which expires by itself: a leftover extension is a
moment in the past, and the watchdog only ever honors one that is *later* than the
budget deadline. No session can inherit another's reprieve.

`make voice-agentcore` needs no tunnel — the hosted agent is inside the VPC and
dials `10.0.1.10:8765` directly. It does need the robot instance running.

**Escape hatch: skipping the tunnel from your laptop.** Everything the agent does
beyond `cmd_vel` — scene grabs, `find`, the greeter — pulls camera frames, and the
SSM tunnel relays every byte through the SSM service, so those go seconds stale.
`make open-ec2` adds a security-group rule for **your current public address only**
and the agent connects straight to the instance.

> ⚠️ **Understand what this publishes before you use it.** While that rule exists,
> an unauthenticated ROS 2 control plane is reachable from the internet and the only
> control is your source IP. Behind office NAT, café Wi-Fi, or CGNAT that is
> everyone sharing the egress, and any of them can drive the robot. Use it on a lab
> robot in a cleared space, and close it immediately after.

```bash
GO2_ACK_UNAUTHENTICATED_BRIDGE=yes make open-ec2
                       # /32 rule on :8765 for this machine; prints the address
make voice-ec2         # agent → ws://<instance public ip>:8765 (resolves it itself)
                       #   flags pass through: make voice-ec2 ARGS=--text-only
make close-ec2         # revoke when done (stop-ec2 and start-ec2 also revoke)
```

`make voice-ec2` refuses to guess an address when the port is closed or the
instance is stopped, and tells you which one it is.

To roll a new image: `make push-image && make restart-ec2`. That pulls over cached
layers and restarts the systemd unit in seconds.

**Watching the video.** Raw camera over the SSM tunnel saturates it and lags, so
the container streams the **COCO-annotated feed to KVS** instead (`KVS_IMAGE_TOPIC=/annotated_image`,
`KVS_SEGMENT_SEC=2`). Watch it in the AWS console — **Kinesis Video Streams →
`go2-robot-01-camera` → Media playback → Live** (auth = your AWS login). Expect
**a few seconds of latency**; KVS is an observation feed, not a control link, so
drive via Lichtblick/telemetry. Over the tunnel, Lichtblick can also show the
low-bandwidth compressed topics `/camera/image_raw/compressed` and
`/annotated_image/compressed`.

## Gotchas

> **Ctrl-C the tunnel before `make stop-ec2`.** Stopping the instance kills the
> SSM session's remote end, but `session-manager-plugin` keeps holding local port
> 9876 — so the next client connects to a dead socket and hangs with no error.
> `make tunnel` refuses to start on a busy port, which is how you'll notice.

> **Fresh public IP on every start, and that's a feature.** Unitree's cloud WAF
> blocks by source IP after a reconnect storm, and a `stop-ec2`/`start-ec2` is the
> only reliable way to clear it. This is why the robot is in a public subnet rather
> than behind the NAT, whose address is stable and would make a block stick — a
> recurring failure on the old Fargate path. Anything that needs to *allowlist* the
> robot's IP would need an Elastic IP, and would give up this escape hatch.

> **The private IP does not change** across stop/start, only on instance
> replacement. That's what makes it safe for `Go2AgentCoreStack` to hardcode.

> **The auto-stop is out-of-band on purpose.** A systemd timer on the box would be
> minute-precise and free, but it can't fire when the box is wedged — which is the
> case that actually runs the bill up. It also would have had to live in user data,
> and a user-data edit *replaces* the instance (cold 14 GB pull). The Lambda cost
> nothing to add to a running instance and can stop one that stopped answering.

> **The watchdog also revokes any `open-ec2` rule before stopping**, mirroring what
> `stop-ec2` does, so an auto-stopped session can't leave a /32 pointed at a stale
> address to come alive again on the next start. Its own decisions are logged one
> line per poll — the log group is in the stack's `AutoStopLogGroupName` output, and
> `aws logs tail <group> --follow` answers both "how long have I got?" and "why did
> it stop?".

> **Don't bump `ImageTag` to deploy a new image.** It lands in user data, and
> cloud-init only runs once per instance, so the stack **replaces** the instance —
> throwing away the cached layers for a cold 14 GB pull. `latest` + `restart-ec2`
> is the intended workflow.

> **Port collision:** the tunnel defaults to local port **9876** because Amazon
> Quick.app (and a locally-run container) can hold ports in the 8765–8770 range and
> shadow it — a client then "connects" to the wrong thing (often a `401`). Override
> with `GO2_LOCAL_PORT=<port> ./scripts/tunnel.sh`.

> **Region:** the AWS CLI honors `AWS_REGION` over `AWS_DEFAULT_REGION`. If your
> shell exports `AWS_REGION` to something else, the scripts look in the wrong
> region. The Makefile pins both; override with `make <target> GO2_REGION=…`.

> **`destroy-ec2` deletes the cached layers too** (the EBS volume goes with the
> instance). Prefer `stop-ec2` between sessions. The ECR repository survives
> everything — it's owned by no stack on purpose.

> **The VPC can't be deleted while anything imports its subnets.** That's the
> intended protection. Tear down in reverse: agent, robot, then network.

## Debugging a robot that won't connect

`make status-ec2` reads `journalctl -u go2` over SSM — that's where boot failures
land (ECR login, secret fetch, docker itself). Container stdout goes to CloudWatch
instead: `aws logs tail /go2/ec2 --follow`.

Read the **first** error, not the loudest:

- `code=1000 "Device not online"` on the *first* attempt means login succeeded and
  the relay answered — the **robot** hasn't registered with Unitree's cloud. Check
  it's powered on, on Wi-Fi with internet, and that the app's Internet/remote
  toggle is on.
- `HTTP 567 = WAF block` appearing *minutes later* is self-inflicted: the retry
  loop against an offline dog tripped it. A WAF-blocked *login* would never reach a
  `webrtc/connect` call at all, which is how you tell them apart. Stop the
  instance (every retry refreshes the block), fix the robot, then start it again
  for a new IP.
- `Robot 0 validated and ready` is the success line to wait for before opening the
  tunnel.

⚠️ **The robot moves for real.** Commands from the agent drive the physical dog
(velocity is clamped, but give it clear space before the first command).
