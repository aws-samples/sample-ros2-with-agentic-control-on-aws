# Voice Control Demo (Nova Sonic)

> ⚠️ **Sample code — not for production use.** This repository is a demonstration
> sample with no AWS SLA or support. Read the
> [non-production disclaimer](../README.md#%EF%B8%8F-sample-code--not-for-production-use)
> and [RESPONSIBLE-AI.md](../RESPONSIBLE-AI.md) before adapting any of it.

Drive the Go2 with natural language — by voice or by typing. An [Amazon Nova
Sonic](https://aws.amazon.com/ai/generative-ai/nova/) speech-to-speech agent
(via [Strands Agents](https://github.com/strands-agents/sdk-python))
interprets commands like *"stand up"*, *"walk forward for two seconds"*, or
*"do a dance"* and routes them to the robot.

There are four entry points — same agent, same tools, different place it runs:

| Entry point | Where the agent runs | Transport | Robot | Unitree credentials | When to use |
|---|---|---|---|---|---|
| `python -m voice_agent.main` | your laptop | ROS2 / `foxglove_bridge` (`ws://localhost:8766`) | real | **none** | The SDK container is running locally; recommended. |
| `mjpython -m voice_agent.main_sim` | your laptop | in-process MuJoCo simulation | **simulated** | **none** | No dog in the room — see [Simulation](#simulation-main_simpy). |
| `python -m voice_agent.main_sonic` | your laptop | direct WebRTC to the robot | real | **required** | No container — drive the dog directly. |
| `python -m voice_agent.agentcore_client` | **AWS** (Bedrock AgentCore Runtime) | WebSocket to the cloud; agent → robot's private IP | real | **none** | The container is on its EC2 instance — see [Hosted on AgentCore](#hosted-on-bedrock-agentcore-runtime). |

The **ROS2** entry point (`main.py`) needs **no Unitree credentials**: it talks
only to the `foxglove_bridge` this SDK's container exposes, and the container
owns the robot connection (LAN + the per-device AES-128 key in
[`docker/.env`](../docker/.env)) — see the main
[developer guide](../DEVELOPER_GUIDE.md#get-the-aes-128-key) for how that key is obtained.

The **simulation** entry point (`main_sim.py`) needs no robot, no container and
no network — only the AWS credentials Nova Sonic already requires.

The **WebRTC** entry point (`main_sonic.py`) connects to the robot itself, so it
needs the Unitree account (remote mode) or the AES-128 key (local mode) — see
[Direct WebRTC mode](#direct-webrtc-mode-main_sonicpy) below.

```
🐕 > stand up
🤖 Lab Lassie is up and ready! 🐾

🐕 > walk forward for 2 seconds
🤖 Lab Lassie walked forward for 2 seconds at medium speed!

🐕 > wave hello
🤖 👋 Lab Lassie waved hello!

🐕 > crouch down
🤖 Lab Lassie is crouched and resting. Safe to power off!

🐕 > what do you see?
🤖 I see a bright office with black rolling chairs by the windows and a long
   desk with monitors — looks like a comfy place to explore!
```

> **"What do you see?"** — the `describe_scene` tool grabs the latest
> `/camera/image_raw` frame off the Foxglove bridge, sends it to Amazon Bedrock
> (Claude Sonnet 4.6, multimodal), and **returns the description text so Nova Sonic
> speaks it in its own voice**. It runs entirely host-side — no container node,
> no separate TTS. Works on the **ROS2 transport** (`python -m voice_agent.main`)
> and in **simulation** (`main_sim.py`, where the frame is rendered from the
> simulated dog's head camera); on the direct-WebRTC path (`main_sonic.py`) the
> tool returns a helpful error, since that's where no camera is available.
> Bedrock access is the same AWS credentials already required for Nova Sonic.

> **"What did you see earlier?"** — every `describe_scene` call snapshots a
> timestamped (image, description) into an in-session memory log. Ask about the
> past (*"what was on the table 30 seconds ago?"*, *"and two minutes ago?"*) and
> the `recall_scene` tool retrieves the snapshot nearest that time and answers
> from it. Ask what **changed** (*"what's different now?"*, *"did anything
> move?"*) and `compare_scenes` sends both the past snapshot and a live frame to
> the model at once and reports the difference. Memory lasts only while the agent
> runs (ROS2 transport only). See the [developer guide](../DEVELOPER_GUIDE.md#what-did-you-see-earlier--robot-memory-temporal-qa) for
> tuning (`SCENE_MEMORY_MAX`, `SCENE_MEMORY_DIR`).

## How it works

The same Nova Sonic agent and Strands tools (`stand_up`, `move`,
`perform_action`, `describe_scene`, `recall_scene`, `compare_scenes`, …) sit on top of one of three interchangeable
robot clients, chosen by which entry point you launch:

```
 you ──voice/text──► voice_agent (Nova Sonic + Strands tools)
                          │
            ┌─────────────┼─────────────────────────────┐
   main.py  │  main_sim.py│                             │  main_sonic.py
   (ROS2)   ▼    (sim)    ▼                             ▼  (WebRTC)
   Go2ROS2Client     Go2SimClient                  Go2Client
        │ Foxglove WS      │ in-process                 │ unitree_webrtc_connect
        ▼ ws://localhost:8766                           ▼ TURN (remote) / AP·STA (local)
   foxglove_bridge    SimDog + MuJoCo             ─────┐
   (SDK ROS2 container)   (no robot)                   │
        │ ROS2 topics                                  │
        ▼                                              ▼
   go2 driver ──LAN + AES-128──►      the dog     ◄──── (direct)
```

The clients share a public method surface
([`Go2ClientProtocol`](client_registry.py)), so the tools don't care which one is
installed (`main.py` / `main_sim.py` / `main_sonic.py` call `set_client()` at
startup). The scene-description tools are shared too, as
[`SceneVisionMixin`](scene_vision.py): a transport only has to answer "give me
your latest camera frame" — decoded off the bridge for the real dog, rendered out
of MuJoCo for the simulated one — and `describe_scene` / `recall_scene` /
`compare_scenes` work identically on both.

[`Go2ROS2Client`](go2_ros2_client.py) maps each command onto ROS2 topics:

| Purpose | Mechanism |
|---------|-----------|
| Velocity move | publish `geometry_msgs/msg/Twist` to `/cmd_vel_out` at ~10 Hz |
| Posture / tricks | publish `go2_interfaces/msg/WebRtcReq` to `/webrtc_req` (`api_id`, JSON `parameter`) |
| State | subscribe `go2_interfaces/msg/Go2State` on `/go2_states` |

Its transport is the [Foxglove WebSocket protocol](https://github.com/foxglove/ws-protocol)
(subprotocol `foxglove.sdk.v1` for foxglove_bridge ≥ 3.3.0), CDR-encoded. This
is **not** rosbridge JSON — `roslibpy` will not work against this bridge.

[`Go2Client`](go2_webrtc_client.py) instead sends the same MCF sport commands
straight to the robot over a WebRTC data channel via `unitree_webrtc_connect`.

[`Go2SimClient`](go2_sim_client.py) has no transport at all: it drives
[`SimDog`](sim_dog.py), a MuJoCo Go2 stepping in a background thread in the same
process. See [Simulation](#simulation-main_simpy).

The agent itself — system prompt, the full tool surface, the Nova Sonic model — lives in
[agent.py](agent.py), which imports no audio or TTY libraries. Everything that
differs between running locally and running in the cloud is the *transport*: which
`BidiInput`/`BidiOutput` channels get handed to `agent.run()`.

| Module | Provides the IO channels for |
|---|---|
| [session.py](session.py) | local terminal — mic (`sounddevice`) + TTY (`termios`) |
| [agentcore_server.py](agentcore_server.py) | AgentCore Runtime — audio over a WebSocket |

## Prerequisites

> Running in [simulation](#simulation-main_simpy)? Skip 1 and 2 — `make voice-sim`
> needs neither the container nor a robot. You still need the AWS credentials
> below.

1. **The SDK container running** with the robot validated (you can drive the
   dog manually first — see the main [developer guide](../DEVELOPER_GUIDE.md#build-and-run)).
   The bridge must be reachable:

   ```bash
   nc -z -v localhost 8766   # expect "succeeded!"
   ```

2. **AWS credentials with Bedrock / Nova Sonic access** in your shell — any of
   `AWS_PROFILE`, SSO, env vars, or instance role. Verify:

   ```bash
   aws sts get-caller-identity
   ```

3. **Python deps** — this agent runs on the **host** (not the container) and
   needs **Python 3.12+** (for `BidiNovaSonicModel`). One `.venv` serves every
   entry point here; the SDK's container requirements are separate and are
   installed into the image, not this venv:

   ```bash
   make venv
   ```

   `pyaudio` needs the `portaudio` system library (`brew install portaudio`).

4. For voice mode, grant your terminal **Microphone** permission
   (macOS: System Settings → Privacy & Security → Microphone).

## Run

From the repo root:

```bash
python -m voice_agent.main                  # voice + text
python -m voice_agent.main --text-only      # skip the mic
python -m voice_agent.main --voice en-us.matthew --log-level INFO
```

No robot to hand? Same thing against a simulated one — see
[Simulation](#simulation-main_simpy):

```bash
make voice-sim
```

Press **SPACE** on an empty prompt to start the mic; press **SPACE** again to
stop and send. Typed lines (with embedded spaces) still work normally — the
toggle only fires when the input buffer is empty. Stdin is read directly from
the controlling TTY in `termios` cbreak mode, so it must be run from a real
terminal (not piped or redirected). `Ctrl+C` exits; the dog is sent
`StandDown` on the way out.

> Terminals don't deliver key-release events, so SPACE is a toggle rather than
> a hold-to-talk button.

## Configuration

All settings are environment variables (see [config.py](config.py)); the
defaults work for the standard local setup. These apply to both entry points:

| Variable | Default | Purpose |
|---|---|---|
| `ROS_BRIDGE_HOST` | `localhost` | foxglove_bridge host (`main.py`) |
| `ROS_BRIDGE_PORT` | `8766` | foxglove_bridge port (`main.py`) |
| `AWS_BEDROCK_PROFILE` | *(unset → default chain)* | AWS profile for Nova Sonic |
| `NOVA_SONIC_MODEL_ID` | `amazon.nova-2-sonic-v1:0` | Bedrock model id |
| `NOVA_SONIC_REGION` | `us-east-1` | Bedrock region (also the default for `SCENE_REGION`) |
| `GUARDRAIL_ID` | *(unset → no guardrail)* | Amazon Bedrock Guardrail applied to every vision call. `Go2GuardrailStack` creates it and every other stack passes it in; set it by hand for a local run (below) |
| `GUARDRAIL_VERSION` | `DRAFT` | Guardrail version. Pin a number for production |
| `SCENE_MEMORY_MAX` | `120` | Snapshots kept in memory |
| `SCENE_MEMORY_DIR` | *(unset → memory only)* | Write snapshot JPEGs to disk. Off by default — these are photographs of people. See [Privacy](#privacy--this-agent-looks-at-people) |
| `SCENE_MEMORY_TTL_SECONDS` | `1800` | How long an on-disk snapshot may live |
| `SCENE_MEMORY_DISK_MAX` | `120` | Hard cap on on-disk snapshots, newest kept |

Unset `GUARDRAIL_ID` means unguarded model calls — fine for a local experiment, not
for anything else. Attach the deployed guardrail to a local run with:

```bash
export GUARDRAIL_ID=$(aws cloudformation describe-stacks --stack-name Go2GuardrailStack \
  --query "Stacks[0].Outputs[?OutputKey=='GuardrailId'].OutputValue" --output text)
```

To target a bridge on another host:

```bash
ROS_BRIDGE_HOST=192.168.x.x ROS_BRIDGE_PORT=8766 python -m voice_agent.main
```

## Simulation (`main_sim.py`)

Drive a **simulated** Go2 with the same agent, prompt and tools — no dog, no
container, no network. Useful for working on the agent when the robot is
elsewhere (or flat), for demoing without a floor to clear, and for reproducing
a behaviour deterministically.

The dog comes from the [Strands Labs robots
project](https://strandsagents.com/docs/labs/robots/) (`strands-robots`), which
supplies the Go2's real MJCF model from [MuJoCo
Menagerie](https://github.com/google-deepmind/mujoco_menagerie), a physics
world, cameras and an interactive viewer. Everything above the joints —
[`sim_dog.py`](sim_dog.py) — is this repo's.

```bash
make voice-sim          # voice + text, with a viewer window
make voice-sim-text     # text only
make sim-demo           # scripted: stand, walk, sit, wave, dance. No agent, no AWS
make sim-demo SAVE=1    # ... and write a head-camera + room-camera PNG per step
```

`make venv` installs everything needed (`strands-robots[sim-mujoco]`, ~200 MB).
The first run clones MuJoCo Menagerie to fetch the Go2's meshes (~40 s, then
cached in `~/.cache/robot_descriptions`).

**macOS needs `mjpython`** (shipped with the `mujoco` wheel, so `.venv/bin/mjpython`
exists after `make venv`) for the viewer window: MuJoCo's passive viewer has to
own the UI thread and plain `python` cannot give it one. The make targets use it
for you. Without it everything still runs — you just can't watch:

```bash
.venv/bin/python -m voice_agent.main_sim --no-viewer --text-only
```

### What is real and what is animated

Be clear about this when demoing, because the difference is visible:

| | |
|---|---|
| **Real physics** | Standing, sitting, lying, gestures, balance, contacts, the camera. The Menagerie Go2's 12 actuators are **torque** motors, so `sim_dog` runs a PD law (`tau = kp*(q*-q) - kd*qdot`, clipped to the model's own ±23.7 N·m / ±45.43 N·m limits) at the 500 Hz physics rate. The dog holds its own weight and settles in stance at base *z* ≈ 0.25 m. Nothing is teleported. |
| **Animated** | Walking. `strands-robots` ships no Go2 locomotion policy (the `mock` policy is a sinusoid; `go2_walk_forward` is an RL benchmark *spec* — a reward to train against, not a gait), so an open-loop trot plays on the legs while the base pose is integrated from the commanded twist and imposed. The dog tracks commanded speed and heading closely and cannot fall over while walking. |

The first attempt injected only base *velocity* and left height and attitude to
physics. It does not survive contact: planted feet resist the imposed velocity,
the friction couple builds a yaw moment, and the dog spins out and lands on its
back about two seconds into "walk forward". A demo that never faceplants
mid-sentence beat a physically-earned gait we don't have. To replace the
animation with a trained policy, swap `_trot_overlay` / `_drive_base` for
`run_policy` (see the lab's [policies](https://strands-labs.github.io/robots/)).

The room has furniture and walls. Walking into them stops the dog short and the
`move` tool reports `blocked: True` with the distance actually covered, which
the agent relays instead of claiming it walked somewhere it didn't.

### What the simulation can't do

These tools come from container nodes, not from the robot, so they return an
error with an explanation rather than silently doing nothing — the agent is told
about them up front in its prompt and explains the gap when asked:

| Tool | Why |
|---|---|
| `get_detections`, `start_tracking`, `find_object`, `greet_visitor`, `start_greeter` | Need `coco_detector_node`. There is no detector in the sim (and no people in the room). `describe_scene` still works. |
| `perform_action("front_flip" \| "back_flip" \| "left_flip" \| "front_pounce")` | Ballistic manoeuvres the real firmware performs with authority the sim has no controller for. Faking them would put the dog through the floor. |

### Simulation settings

All optional (see the `SIM_*` block in [config.py](config.py)):

| Variable | Default | Purpose |
|---|---|---|
| `SIM_ROBOT` | `unitree_go2` | Any robot in the lab's registry, though the poses are tuned for the Go2's joints |
| `SIM_SCENE` | `lab` | `lab` (walled room, workbench, chairs, props) or `empty` (bare floor) |
| `SIM_KP` / `SIM_KD` | `60` / `5` | Joint PD gains |
| `SIM_GAIT_FREQ_HZ` | `2.0` | Trot cycle frequency |
| `SIM_CAMERA_FOV` | `78.0` | Head-camera field of view, degrees |
| `SIM_BODY_RADIUS` | `0.60` | Standoff from furniture. Deliberately bigger than the dog: the camera sits 0.28 m ahead of the base, and a body-tight radius parks the lens flush against whatever it walked up to, so every frame comes back a flat wall of colour |

## Direct WebRTC mode (`main_sonic.py`)

Use this only if you're **not** running the SDK container and want the agent to
drive the robot directly. It needs the extra WebRTC dependency (commented out in
`requirements.in` by default):

```bash
.venv/bin/pip install 'unitree-webrtc-connect==2.2.0'
```

Then set the Unitree connection variables and launch:

```bash
# Remote mode — via Unitree's TURN server, works from any network
export UNITREE_CONN_TYPE=remote
export UNITREE_EMAIL=you@example.com
export UNITREE_SERIAL=<robot serial>
export UNITREE_PASSWORD='...'          # or: UNITREE_SECRET_NAME + AWS_SECRETS_PROFILE
python -m voice_agent.main_sonic

# Local mode — same network as the robot (AP hotspot or STA)
export UNITREE_CONN_TYPE=local
export UNITREE_LOCAL_MODE=sta           # or "ap"
export UNITREE_ROBOT_IP=192.168.x.x
export UNITREE_AES_128_KEY=<32 hex>     # firmware ≥1.1.15; same key as docker/.env
python -m voice_agent.main_sonic
```

| Variable | Mode | Purpose |
|---|---|---|
| `UNITREE_CONN_TYPE` | both | `remote` (TURN) or `local` (AP/STA) |
| `UNITREE_EMAIL` / `UNITREE_SERIAL` | remote | Unitree account + robot serial |
| `UNITREE_PASSWORD` | remote | password (or use the secret below) |
| `UNITREE_SECRET_NAME` / `AWS_SECRETS_PROFILE` | remote | Secrets Manager fallback for the password |
| `UNITREE_LOCAL_MODE` | local | `ap` (robot hotspot) or `sta` (shared WiFi) |
| `UNITREE_ROBOT_IP` | local STA | robot's IP |
| `UNITREE_AES_128_KEY` | local | per-device key (firmware ≥1.1.15) |

> Close the Unitree mobile app first — only one WebRTC client can connect at a
> time.

## Hosted on Bedrock AgentCore Runtime

When the robot container runs on its EC2 instance in the shared VPC (see
[deployment](../deployment/README.md)), the agent can move into the cloud
too. The microphone is inherently on your laptop, so it stays there — but
everything else (Nova Sonic session, every tool, the robot link) runs in AWS:

```
 your laptop                             AWS
 ───────────                             ───
 agentcore_client.py
   mic ──16 kHz PCM──┐
                     │  wss://bedrock-agentcore.<region>.amazonaws.com
                     │       /runtimes/<arn>/ws     (SigV4 pre-signed)
                     ▼
              AgentCore Runtime ── in the Go2 VPC ──► 10.0.1.10:8765
              (BidiAgent + Nova Sonic,                       │
               agentcore_server.py)                          ▼
   speaker ◄── spoken reply ◄──┘                     foxglove_bridge
                                                     (robot instance) ──► the dog
```

Why this is different from `make voice-cloud`: the agent runs **inside the VPC**,
so it reaches the bridge at its fixed private IP directly and the SSM tunnel drops
out of its path entirely. `scripts/tunnel.sh` remains only for viewing topics
locally in Lichtblick or running the laptop-side agent. There is no load balancer
in between — an instance's private IP survives stop/start, so there is nothing
unstable to hide behind one. Nothing is exposed to the internet; access is IAM
(who can open a WebSocket to the runtime).

### Deploy

```bash
# 1. Deploy (builds the ARM64 image, creates the runtime in the Go2 VPC)
make deploy-agentcore

# 2. Start the robot instance if it isn't already running
make start-ec2

# 3. Install the host-side deps (once) — same .venv as every other entry point
make venv

# 4. Talk to the dog — no ARN to paste; it reads the stack's AgentRuntimeArn
#    output using your terminal's current AWS profile and region.
.venv/bin/python -m voice_agent.agentcore_client
```

The client's UI matches the local agent: **SPACE** on an empty prompt toggles
the mic, typed lines are sent as text, `Ctrl+C` exits. Both voice and text work.

To point at a different runtime, override in this precedence order:
`--agent-arn` → `$AGENT_ARN` → the stack lookup (`--stack`, or
`$GO2_AGENTCORE_STACK`, default `Go2AgentCoreStack`).

Run it from a terminal whose AWS profile and region match the deployed runtime —
both the stack lookup and the WebSocket signing use them directly. A region
mismatch shows up as an HTTP 403 on connect.

> **Nova Sonic needs a continuous audio stream.** It infers end-of-utterance
> from trailing silence, so the client always feeds audio — real mic frames when
> the mic is on, silence otherwise (including in `--text-only`, which has no mic
> at all). Let that stream stall and the model never closes your turn: commands
> appear to do nothing until the *next* time you speak, and transcripts run
> together. This is why both [session.py](session.py) and
> [agentcore_client.py](agentcore_client.py) send silence when idle.

### Test the server locally first

The container is a plain WebSocket server, so you can run it on your Mac against
the local SDK container before deploying anything:

```bash
# Terminal 1 — the agent server (needs the bridge reachable)
.venv/bin/python -m voice_agent.agentcore_server

# Terminal 2 — the client, pointed at localhost instead of AWS
.venv/bin/python -m voice_agent.agentcore_client --url ws://localhost:8080/ws
```

`curl localhost:8080/ping` returns the health status AgentCore polls.

### Configuration

Set on the runtime by [go2-agentcore-stack.ts](../deployment/lib/go2-agentcore-stack.ts).
The knobs are CDK **context** values with defaults in
[cdk.json](../deployment/cdk.json), read at synth time — there is no wrapper
script and no `--parameters` to remember:

| Context key | Default | Purpose |
|---|---|---|
| `novaSonicVoice` | `en-us.matthew` | Nova Sonic voice |
| `deviceId` | `go2-robot-01` | Which robot's camera stream the container writes to (see below) |
| `modelRegion` | *(the stack's own region)* | Region for Nova Sonic + the scene model. `us-east-1` has both, so the default keeps the demo single-region — set it only when deploying where Nova Sonic is unavailable |

Override for one deploy without editing anything:

```bash
cd deployment && npm run deploy:agent -- --context novaSonicVoice=en-us.joanna
```

The stack's region is *not* set in `cdk.json` on purpose — it follows
`AWS_REGION`, which the [Makefile](../Makefile) pins (`GO2_REGION`, default
`us-east-1`), so `make deploy-agentcore GO2_REGION=…` keeps working and the ECR
image asset cannot land in a different region from the stack.

`deviceId` is the demo's robot identifier. It names one thing: the KVS stream
`<deviceId>-camera`, which `Go2KVSStack` creates and the container's
`kvs_producer_node` uploads to.

Agreement is structural: [bin/app.ts](../deployment/bin/app.ts) resolves it once
from context and hands it to `Go2KVSStack` and `Go2Ec2Stack` as a required prop,
so `--context deviceId=…` moves both together. (It used to be a separate `DeviceId`
CloudFormation parameter per stack — one value per stack to keep in sync by hand,
and a mismatch was silent.) This stack doesn't take it at all anymore: the agent
reaches the robot over `ROS_BRIDGE_HOST` and has no tools that read the stream.

Leave it at the default unless you're running more than one dog — changing it
**replaces** the KVS stream, since the name is that resource's physical ID.

The container also reads `STAND_DOWN_ON_EXIT` (default `false`). Unlike the local
agent, the hosted one does **not** crouch the dog when a session ends — an
operator watching the robot shouldn't have it sit down because their WiFi
dropped. Set it to `true` to restore the local behaviour.

### Wire protocol

JSON text frames both ways; audio is base64 16 kHz/16-bit/mono PCM (Nova Sonic's
contract). Useful if you want to build a different client — a browser, say:

| Direction | Frame |
|---|---|
| client → server | `{"type":"audio","audio":"<base64 pcm>"}` |
| client → server | `{"type":"text","text":"stand up"}` |
| server → client | `{"type":"audio","audio":"<base64 pcm>"}` |
| server → client | `{"type":"transcript","role":"user"\|"assistant","text":"…","is_final":bool}` |
| server → client | `{"type":"tool","name":"stand_up"}` |
| server → client | `{"type":"interruption"}` — clear your playback buffer |
| server → client | `{"type":"error","message":"…"}` / `{"type":"ready"}` |

### Gotchas

- **Nova Sonic streams cap at 8 minutes.** A long session will hit this; the
  Strands model layer reconnects, but expect a pause.
- **ARM64 is required** by AgentCore Runtime. CDK builds the image with the right
  platform; a hand-rolled `docker build` on an x86 host needs
  `--platform linux/arm64`.
- **`kvs_describe` / `kvs_compare_scenes` do not work here.** They ship in the image,
  but the runtime's ENIs are in isolated subnets with no NAT and the VPC has no Lambda
  interface endpoint, so the invoke can't leave. They return an error and the agent
  falls back to `describe_scene` / `compare_scenes`. Use `make voice` to demo the KVS
  route. (`ffmpeg` is *not* in this image — frame extraction happens inside the
  Lambda, and the camera upload happens in the robot container.)
- **Scene memory is per-session.** `recall_scene` / `compare_scenes` build their
  memory in-process, so it's scoped to one microVM session and doesn't persist.
- If the client reports *"Foxglove bridge unreachable"*, the robot instance is
  stopped — run `make start-ec2` and give it ~40 seconds. If it is running, check
  `make status-ec2`: the container may be up but unable to reach the dog.

## Safety

Three controls, in code rather than in prose:

- **Velocities are clamped** in [config.py](config.py)
  (`MAX_LINEAR_VELOCITY` 0.5 m/s, `MAX_ANGULAR_VELOCITY` 0.8 rad/s). No tool
  argument can command a faster robot.
- **An unknown `move` direction is an error**, never a default. Nothing that
  actuates falls back to moving.
- **Flips, jumps and pounces take two tool calls in two turns.** `perform_action`
  refuses them; reaching one needs `request_dangerous_action`, then the user
  actually saying out loud that the space is clear, then
  `confirm_dangerous_action` naming the same action within 60 seconds. A single
  utterance — including one injected through the camera — cannot reach a flip.
  See the docstring at the top of [tools.py](tools.py).

They still need ≥2 m of clear space in every direction and a flat, non-slippery
surface. The physical remote is the real stop button.

## Privacy — this agent looks at people

`describe_scene`, `recall_scene`, `compare_scenes` and greeter mode all send camera
frames to a vision model, and greeter mode does it **automatically**, to whoever
walks up, without being asked.

- Frames are held in memory (`SCENE_MEMORY_MAX`, default 120) for the life of the
  process, and written to disk **only** if you set `SCENE_MEMORY_DIR` — off by
  default. When on, files are 0600 in a 0700 directory and swept after every write
  against `SCENE_MEMORY_TTL_SECONDS` (default 30 min) and `SCENE_MEMORY_DISK_MAX`.
- `GREETER_VISITOR_PROMPT` forbids identification: position and one obvious visual
  detail such as clothing colour, and explicitly not name, age, gender or personal
  characteristics. The Bedrock Guardrail anonymises `NAME`, `EMAIL` and `PHONE` in
  model output as a second layer.
- Transcribed speech is never logged. Tool call sites log lengths and flags only.

Establishing a lawful basis, a retention period and notice to the people in the
room is yours. The full picture is in
[Imagery, people and privacy](../README.md#imagery-people-and-privacy), and the
model-safety side is in [RESPONSIBLE-AI.md](../RESPONSIBLE-AI.md).
