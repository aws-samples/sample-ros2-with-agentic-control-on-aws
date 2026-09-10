# Go2 Cloud — CDK Infrastructure

> ⚠️ **Sample code — not for production use.** This repository is a demonstration
> sample with no AWS SLA or support. Read the
> [non-production disclaimer](../README.md#%EF%B8%8F-sample-code--not-for-production-use)
> and [RESPONSIBLE-AI.md](../RESPONSIBLE-AI.md) before adapting any of it.

The four stacks that run the robot and its voice agent in AWS, plus the Kinesis
Video Stream the robot uploads its camera to.

## Architecture

```
Robot container (Go2Ec2Stack, EC2)
  ├─ coco_detector ──► /annotated_image ──► kvs_producer_node ──► KVS stream
  │                                                                  │
  │                                                you watch it over HLS
  │                                                                  │
  │                                          SceneDescriber Lambda ──┘
  │                                          (GetClip → Bedrock)
  │                                                  ▲
  │                              kvs_describe / kvs_compare_scenes,
  │                                        from a LOCAL agent only
  │
  └─ foxglove_bridge :8765 ◄──── Go2AgentCoreStack (hosted voice agent)
                                          │
                                 Nova Sonic + Bedrock (direct)
```

Mostly KVS is an **observation feed**: frames go up, humans watch them. The voice
agent answers *"what do you see?"* by grabbing a frame off the foxglove bridge and
calling Bedrock itself, never touching KVS.

The exception is `SceneDescriber`, behind two voice tools:

| Tool | Event | What it does |
|---|---|---|
| `kvs_describe` | `{question}` | Newest frame → Bedrock describes it. Same answer as the agent's own `describe_scene`, by a longer route — worth showing, not worth defaulting to. |
| `kvs_compare_scenes` | `{question, seconds_ago}` | Frame from `seconds_ago` **and** the newest one, both to Bedrock → what changed. |

`kvs_compare_scenes` is the one that earns its keep. The agent's local
`compare_scenes` compares against a snapshot it happened to take during the session,
so it is blind to any moment nobody asked about. This reads the stream's 24-hour
archive, so *"what changed since this morning?"* works — that is what retention is
for, and it is the capability the old (never-working) `SceneRecaller` was reaching
for, done as a comparison rather than as a description of a lone past frame.

**Only reachable from a local agent run:** the hosted runtime's ENIs are in isolated
subnets with no NAT and this VPC has no Lambda interface endpoint, so the invoke
can't leave. Adding that endpoint is the only thing standing in the way; the IAM
grant is already in `Go2AgentCoreStack`.

> **History:** this stack used to hold a much larger pipeline — three
> container-image Lambdas (describe / recall / monitor) behind IoT topic rules, a
> DynamoDB baseline table, a 1-minute EventBridge schedule, and an MQTT→ROS2 bridge
> that published text to a `/tts` node for the robot to speak. Everything but the
> describer is gone. `SceneMonitor` sat at reserved concurrency 0 while its schedule
> kept firing — 1,440 events/day received, 1,440 dropped, for months.
> `SceneRecaller` never produced a single log stream, and the agent's `recall_scene`
> never used it anyway (it works off in-session snapshot memory) — what it was
> reaching for now lives in `kvs_compare_scenes`. Nova Sonic's speech-to-speech had
> already made the `/tts` leg redundant, so the describer now just returns its answer
> instead of publishing it.

## Stacks

| Stack | What it is |
|---|---|
| **`Go2KVSStack`** | The KVS stream `<deviceId>-camera` (24 h retention) and the `SceneDescriber` Lambda. Serverless, no VPC. |
| **`Go2NetworkStack`** | The shared VPC. Public subnets hold the robot, isolated subnets hold the agent's ENIs. No NAT — VPC endpoints instead. |
| **`Go2Ec2Stack`** | The robot: one EC2 instance running the SDK container at a fixed private IP, no public endpoint. Replaced a Fargate stack that cost ~$57/mo idle and re-pulled 14 GB on every start. |
| **`Go2AgentCoreStack`** | The hosted voice agent, same VPC, dialling the robot's private IP directly — no load balancer, no tunnel. |

## Deploy

Deploy them in order with `make deploy-cloud`, or individually — see
[the AWS deployment guide](../AWS_DEPLOYMENT.md#one-time-setup-operators).

```bash
cd deployment
# --frozen-lockfile, not a bare install: it installs exactly what pnpm-lock.yaml
# records and fails rather than silently resolving something else. Every version
# in package.json is pinned exactly, so a deploy from this commit synthesizes the
# same template.
pnpm install --frozen-lockfile
npx cdk bootstrap   # first time only
npx cdk deploy Go2KVSStack
```

> **Note:** Docker must be running — CDK builds an amd64 Lambda container image for
> `SceneDescriber`. With a non-Docker OCI runtime, point CDK at it with
> `CDK_DOCKER=<binary>`.

Override the device ID. It's shared context, not a per-stack parameter — one value
reaches both stacks that need it (`Go2KVSStack` creates the `<id>-camera` stream,
the robot container writes to it), so they can't drift apart:

```bash
npx cdk deploy Go2KVSStack --context deviceId=my-robot
```

> Changing it **replaces** the KVS stream — the name is its physical ID. Treat it
> as adding a robot, not renaming one. The default (`go2-robot-01`) lives in
> [bin/app.ts](bin/app.ts); [cdk.json](cdk.json) holds the overridable value.

## Sending video to KVS

Nothing to set up — the robot container does it. [`kvs_producer_node`](../go2_robot_sdk/go2_robot_sdk/kvs_producer_node.py)
subscribes to an image topic, encodes H.264 MKV with ffmpeg, and uploads via
PutMedia with SigV4. [go2-ec2-stack.ts](lib/go2-ec2-stack.ts) launches it with
`kvs:=true` and points it at the detector's overlay feed:

```
KVS_IMAGE_TOPIC=/annotated_image
KVS_SEGMENT_SEC=2
KVS_STREAM_NAME=<deviceId>-camera
```

Other knobs: `KVS_FPS` (15), `KVS_WIDTH` (1280), `KVS_HEIGHT` (720),
`KVS_QOS_RELIABLE`. Locally, add `kvs:=true` to the launch command and set
`KVS_ENABLED=true` in `docker/.env`.

To watch the stream, see [Watching the KVS video stream](../README.md#watching-the-kvs-video-stream).

### Without a robot

[`edge/kvs_producer/`](edge/kvs_producer/) pushes your Mac's webcam into the same
stream — useful for checking the stream, IAM, and the HLS viewer before the dog is
on the network:

```bash
pip install 'opencv-python==4.11.0.86' 'boto3==1.43.88' 'requests==2.34.2'
DURATION=30 KVS_STREAM_NAME=go2-robot-01-camera AWS_DEFAULT_REGION=us-east-1 \
  python3 edge/kvs_producer/push_to_kvs.py
```

## Prerequisites

- AWS CLI configured with credentials
- Node.js 18+ and **pnpm** (`corepack enable pnpm`, or see pnpm.io/installation).
  `pnpm-lock.yaml` is the only lockfile in `deployment/`; using npm here would
  write a second one, so a `preinstall` guard rejects it.
- No global CDK install needed — `npx cdk` runs the `aws-cdk@2.1139.0` pinned in
  `package.json`.
- Docker (the agent runtime image is built locally)
- Bedrock model access for Nova Sonic and Claude Sonnet in your region
