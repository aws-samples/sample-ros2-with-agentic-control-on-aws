# ROS2 with Agentic Control on AWS

> ## ⚠️ Sample code — not for production use
>
> This repository is a **demonstration sample** published for learning purposes.
> It is not production-ready, and it is not covered by an AWS SLA or support
> agreement. Before adapting any part of it for production, review and harden at
> minimum:
>
> - **The Foxglove bridge (tcp/8765) has no authentication.** Anyone who can reach
>   it has full control of the robot — arbitrary topic publish, `/cmd_vel` and
>   sport commands included. Use the SSM tunnel (`make tunnel`), which gates access
>   by IAM. Never expose the port directly; `make open-ec2` does, and refuses to
>   run without an explicit acknowledgement for exactly that reason.
> - **IAM policies use model-family and resource-prefix wildcards** to keep the
>   sample deployable as model IDs change. Scope them to exact ARNs. Each one's
>   reasoning is recorded as a cdk-nag suppression in `deployment/lib/`.
> - **Amazon Bedrock Guardrails are configured with sample settings** and pinned to
>   the `DRAFT` version. Review the content filters and PII policy against your own
>   risk posture and pin a numbered version. See `deployment/lib/go2-kvs-stack.ts`.
> - **The system records and processes imagery of people.** Establish a lawful
>   basis, a retention limit, and notice before deploying anywhere people are
>   present — see [Imagery, people and privacy](#imagery-people-and-privacy).
> - **Model output drives physical motion.** A language model chooses which robot
>   commands to issue. See [RESPONSIBLE-AI.md](RESPONSIBLE-AI.md) for the intended
>   uses, the known failure modes, and the controls that are and are not in place.
> - **Dependencies are pinned, and the Python pins are CVE-checked — but that is
>   not a full scan.** `make audit` checks every pinned Python package against OSV
>   and runs `npm audit`; it does **not** cover OS packages from the container base
>   images, or the vendored `libvoxel.wasm`, whose problem is undetermined
>   provenance rather than a CVE (see
>   [THIRD-PARTY-LICENSES](THIRD-PARTY-LICENSES)). Generate an SBOM and scan that
>   before you build for production, and re-run it — a clean result today is a
>   statement about today.
> - **There is no spend limit on a session.** Token budgets are bounded per call,
>   not per conversation — see [Cost and rate limits](#cost-and-rate-limits).
>
> You are responsible for the security and compliance of anything you derive from
> this sample.

Drive a physical Unitree Go2 robot dog by voice, from your laptop, with the
robot's brain running in AWS. Ask it to stand up, walk, dance, or *"what do you
see?"* — an Amazon Nova Sonic speech-to-speech agent interprets the request and
an Amazon Bedrock vision model describes what the robot's camera sees.

The ROS 2 container runs on an **EC2 instance in AWS**. There are two ways to
connect it to the dog, and you can switch between them in about 30 seconds:

- **Cloud mode** — EC2 reaches the dog through Unitree's cloud. Nothing runs
  locally; you just open an IAM-gated tunnel and talk.
- **AP mode** — the dog runs its own Wi-Fi hotspot and your laptop bridges it to
  EC2. Slightly more setup, but **zero calls to Unitree's cloud**.

See [Two ways to run it](#two-ways-to-run-it) for both architectures.

> ⚠️ **The robot moves for real.** Commands drive the physical dog (velocity is
> clamped, but give it clear space before your first command).

**Watch it run** (2 min, YouTube):

[![Unitree Go2 cloud voice demo — voice commands driving the physical robot](https://img.youtube.com/vi/Zu9zC-Oa7qM/hqdefault.jpg)](https://www.youtube.com/watch?v=Zu9zC-Oa7qM)

For the story behind it, read the write-up on AWS Builder Center:
**[Run a ROS 2 robot stack on Amazon EC2 and drive it with a voice agent](https://builder.aws.com/content/3ISz7ce0hvmQ8i4d9BIhzntb6fQ/run-a-ros-2-robot-stack-on-amazon-ec2-and-drive-it-with-a-voice-agent)**.

**This page is the demo.** For everything else — running locally on macOS with
Docker, SLAM/Nav2, mapping, KVS camera streaming, the full sport-mode command
table, and troubleshooting — see **[DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md)**.
Deploying and operating the cloud stacks is
**[AWS_DEPLOYMENT.md](AWS_DEPLOYMENT.md)**.

---

## What you need

| Item | Notes |
| --- | --- |
| An AWS account | With permissions to deploy the stacks in [deployment/](deployment/) (VPC, EC2, ECR, Secrets Manager, Bedrock AgentCore, Kinesis Video Streams) and to invoke Amazon Bedrock models in your region |
| The robot powered on | On your Wi-Fi for cloud mode, on its own hotspot for AP mode — see [Put the robot in the right mode](#put-the-robot-in-the-right-mode). Not needed at all in [simulation](#no-dog-run-the-same-agent-in-simulation) |
| `make` and the AWS CLI v2 | Plus the Session Manager plugin. Every step below is a `make` target; run `make` for the full list |
| Python 3.12+ | For the voice agent venv (`brew install python@3.13`) |
| AWS CLI v2 + Session Manager plugin | [install the plugin](https://docs.aws.amazon.com/systems-manager/latest/userguide/session-manager-working-with-install-plugin.html) |

### Put the robot in the right mode

The two run modes want the dog on different networks. **Cloud mode** needs it on
Wi-Fi with internet — the steps are right below. **AP mode** needs it on its own
hotspot instead; that setup lives with [Run it — AP mode](#run-it--ap-mode). Either
way, **close the Unitree app when you're done** — the robot accepts only one WebRTC
client at a time, and if the app stays connected the demo silently fails.

For cloud mode the dog reaches the container through Unitree's TURN server, so it
needs internet — but it does **not** need to be on the same network as your laptop.

Power on the dog, wait for it to finish booting, then in the **Unitree mobile
app**:

1. Tap **Device**.
2. Tap **RobotDog Settings**.
3. Tap **Wi-Fi mode**.
4. Enter the **Wi-Fi name and password**, and toggle on **Internet remote
   connection**. ← Required; this is what lets the cloud container reach the dog.
5. Tap **Next** and wait for it to connect.
6. **Close the Unitree app.**

### Clone the repo and build the voice agent venv

```bash
git clone https://github.com/aws-samples/unitree-go2-ros2-aws.git
cd unitree-go2-ros2-aws
```

The agent runs on your laptop (not in the cloud container) and needs **Python
3.12+**. One venv covers every host-side entry point; the SDK's ROS2 deps live
in the container and never touch it:

```bash
make venv
```

> `pyaudio` needs the PortAudio system library first: `brew install portaudio`.

---

## Two ways to run it

Both give you the same voice agent, the same Lichtblick view and the same KVS
pipeline. The only difference is **who holds the robot's WebRTC link** — and that
one difference decides whether Unitree's cloud is in the path at all.

### Cloud mode — EC2 owns the robot link

![Architecture, cloud mode: the user drives Lichtblick and the AgentCore voice
agent; the ROS2 container in the VPC connects directly to the robot and streams
camera video to Kinesis Video Streams.](docs/Architecture%20Diagram-EC2.png)

The instance logs in to Unitree's cloud for a TURN token and reaches the dog over
a WebRTC data channel, so the robot never joins your network. Your laptop only
opens an SSM tunnel.

**Use it when** the dog has internet and you want the least setup — no Docker, no
robot Wi-Fi, no Unitree credentials on your machine.

**The catch:** that cloud login is the one leg outside your control. When Unitree's
WAF starts answering `567`, every ROS topic goes silent while the bridge itself
still looks healthy.

### AP mode — your laptop owns the robot link

![Architecture, AP mode: the robot connects to the laptop over its own Wi-Fi
hotspot; the laptop bridges into the ROS2 container in the VPC, which still feeds
Lichtblick, AgentCore and Kinesis Video Streams.](docs/Architecture%20Diagram-AP.png)

The dog runs its own hotspot and talks to nothing but your laptop. A host process
([`scripts/go2_ap_bridge.py`](scripts/go2_ap_bridge.py)) holds the WebRTC link and
republishes into EC2's ROS graph through the same tunnel, so everything downstream
— Lichtblick, the voice agent, KVS, the GPU detector — is unchanged.

**Use it when** Unitree's cloud is blocking you, or the dog has no internet.

**The catch:** your laptop has to be on the dog's hotspot *and* still reach AWS, so
it needs a second interface (USB ethernet or tethering). SLAM and Nav2 are not
available in this mode.

|  | Cloud mode | AP mode |
| --- | --- | --- |
| Holds the robot link | EC2 | your laptop |
| Calls Unitree's cloud | yes | **no** |
| Robot needs internet | yes | no |
| Laptop needs the robot's Wi-Fi | no | yes (plus a second interface for AWS) |
| Local container | no | no |
| SLAM / Nav2 | yes | no |
| Switch to it with | `make ec2-remote` | `make ec2-bridge` |

Switching is a container restart on the instance (~30s), not a redeploy. In AP mode
the instance does not even fetch the Unitree credentials, so it *cannot* contact
Unitree's cloud — `make status-ec2` confirms it.

### No dog? Run the same agent in simulation

```bash
make venv        # once
make voice-sim   # voice + text, with a MuJoCo window
```

A simulated Go2 — real MJCF model, real physics, a head camera that
`describe_scene` reads — driven by the *same* agent, prompt and tools. No robot,
no container, no AWS beyond the Bedrock credentials Nova Sonic already needs, and
nothing deployed. `make sim-demo` runs a scripted stand/walk/sit/wave without even
those.

Poses and balance are physically simulated; the walking gait is animated, and the
detector and Nav2 tools are honest about being absent. Details, and what to say
when demoing it, are in
[voice_agent/README.md](voice_agent/README.md#simulation-main_simpy).

---

## Run it — cloud mode

```bash
make start-ec2      # boot the instance; ready in ~40s (image is cached on its volume)
make ec2-remote     # EC2 owns the robot link (the default — only needed if you switched)
make tunnel         # IAM-gated SSM port-forward, localhost:9876 -> bridge. LEAVE OPEN
```

Then in a second terminal:

```bash
make voice-cloud    # Nova Sonic agent against the tunnel
```

Skip `start-ec2` if a teammate already started it — check with `make status-ec2`.

> **Local port 9876, not 8765:** Amazon Quick.app squats ports 8765–8770 on some
> Macs and answers the WebSocket with a `401`, so clients "connect" to the wrong
> thing. `make tunnel` fails loudly rather than silently if 9876 is also busy.

The tunnel above is **the supported way in**, and the only one this guide
recommends: it opens no port, and who may use it is an IAM decision.

<details>
<summary><b>Escape hatch: connecting directly, without the tunnel (read the risk first)</b></summary>

The SSM tunnel relays every byte through the SSM service, which the camera and
point cloud feel. You can connect straight to the instance instead — but
understand what that publishes:

> ⚠️ **`make open-ec2` puts an unauthenticated ROS 2 control plane on a public
> address.** `foxglove_bridge` has no authentication of any kind: anyone who
> reaches tcp/8765 can publish arbitrary topics, including `/cmd_vel` and sport
> commands, and drive the robot. The only control is your source IP — and behind
> office NAT, café Wi-Fi, or CGNAT, "your address" is everyone sharing that
> egress. Use it on a lab robot in a cleared space, or not at all.

It refuses to run until you say so explicitly:

```bash
GO2_ACK_UNAUTHENTICATED_BRIDGE=yes make open-ec2   # /32 rule for THIS machine's public IP
make voice-ec2      # agent -> ws://<instance>:8765, no tunnel
make close-ec2      # revoke as soon as you are done
```

`make stop-ec2` and `make start-ec2` both revoke the rule automatically, and so
does the 4-hour auto-stop watchdog — but `make close-ec2` is the one you should
actually type.

</details>

## Run it — AP mode

**First, put the dog on its own hotspot.** Here the dog is the access point: it
serves its own network on `192.168.12.0/24` at `192.168.12.1` and reaches nothing
else. Join your laptop's **Wi-Fi** to it (the SSID and key are on the label in the
battery bay, and visible in the app), and keep a **second interface** for internet —
USB ethernet or phone tethering — because the hotspot has no uplink and AWS still
has to be reachable. Then **close the Unitree app**: only one WebRTC client at a
time, and the bridge below is that client.

```bash
make ap-probe       # 30s sanity check: dog reachable on its AP, and you're dual-homed
make start-ec2
make ec2-bridge     # EC2 stops calling Unitree and consumes only
make tunnel         # LEAVE OPEN
```

> **First time on a given image:** bridge mode needs the container image to contain
> the `CONN_TYPE=bridge` support. `make deploy-ec2` deploys the *stack*, not the
> image — so if `make ec2-bridge` reports the unit failing with a ROS parameter
> tuple error and `Robot IPs: []`, run `make push-image && make restart-ec2` once.
>
> The same applies, less loudly, to object detection. The laptop sends JPEG frames
> (`/camera/image_raw/compressed`), and only an image whose `robot.launch.py`
> derives the detector's input topic from `CONN_TYPE` picks them up. On an older one
> the container comes up **healthy with no detections at all** — no boxes in
> Lichtblick, no KVS stream, and *"find a person"* / *"follow me"* silently do
> nothing, while *"what do you see?"* still works because it bypasses the detector.
> Same fix: `make push-image && make restart-ec2`.

Then in a second terminal:

```bash
make ap-bridge      # laptop holds the dog's link and feeds EC2. LEAVE OPEN
```

And in a third:

```bash
make voice-cloud    # same agent, same port as cloud mode
```

The tunnel carries the camera *and* the point cloud in this mode, so the direct
path is worth more here than in cloud mode — at the same cost. Re-read the warning
under [Run it — cloud mode](#run-it--cloud-mode) before using it: this exposes the
unauthenticated bridge to your public address.

```bash
GO2_ACK_UNAUTHENTICATED_BRIDGE=yes make open-ec2
make ap-bridge DIRECT=1     # publish straight to the instance, no SSM relay
make voice-ec2
make close-ec2              # as soon as you're done
```

`make ap-bridge` prints a heartbeat so you can see data flowing in both directions:

```
[i] in: frames=42 state=21 low=21 tf=105 joints=21 lidar=8(4211pts/38k map) | out: published=218 | back: cmd_vel=130 req=2 | soc=87%
```

`in:` is what it receives from the dog, `out:` what it publishes to EC2, and
`back:` the commands arriving from the agent. A stream stuck at `0` is the thing to
look at.

## Talking to it

Press **SPACE** on an empty prompt to start talking, **SPACE** again to send. Typed
lines work too.

> Start every session with *"stand up"*, then *"balance stand"* — the dog ignores
> movement commands until it's in BalanceStand.

**Things to try:**

*Movement and tricks*

| Say | What happens |
| --- | --- |
| *"stand up"* / *"balance stand"* | Rises, then engages active balance (required before walking) |
| *"walk forward for two seconds"* | Drives at a clamped velocity for a set duration |
| *"turn left"* / *"stop"* | Steer, or halt immediately |
| *"do a dance"* | Dance routine — say *"dance two"* for the other one |
| *"wave hello"* | Paw wave |
| *"stretch"* / *"sit"* / *"lie down"* | Other set-piece motions |

*Vision (camera → Bedrock, straight from the bridge)*

| Say | What happens |
| --- | --- |
| *"what do you see?"* | Grabs a live frame, Bedrock describes it, Nova Sonic speaks it |
| *"what's on the table?"* | Same, but answers your specific question |
| *"what did you see a minute ago?"* | Recalls the nearest snapshot from session memory |
| *"what's different compared to a minute ago?"* | Puts the old snapshot **and** a live frame in front of the model and reports what changed |

*Vision — the cloud route (via the recorded video stream)*

| Say | What happens |
| --- | --- |
| *"kvs describe scene"* | A Lambda pulls the newest frame out of the KVS stream → Bedrock describes it |
| *"kvs compare scene with 10 minutes ago"* | Pulls a frame from **the archive** plus a live one, and reports what changed |

> `kvs describe scene` is the same answer as *"what do you see?"* by a longer route —
> the demo beat is to ask both and point out they agree.
>
> `kvs compare scene` is **not** just a slower copy. The local *"what's different"*
> compares against a snapshot taken earlier **in this session**, so it can't answer
> about a moment nobody was looking at. The KVS one reads 24 hours of recorded video,
> so *"what changed since this morning?"* and *"did anything move while I was out?"*
> work — that's what the retention is for.
>
> **Both need the local agent** (`make voice`). The hosted agent runs in subnets with
> no route out to Lambda, so these fail there — it says the cloud path is unavailable
> and answers from the live camera instead.

---

## Watch it in Lichtblick

[Lichtblick](https://github.com/lichtblick-suite/lichtblick) is an OSS robotics
viewer (a BMW-maintained fork of Foxglove Studio v1) for telemetry and teleop.
**No install needed** — open it in your browser:

**https://lichtblick-suite.github.io/lichtblick/**

With the tunnel from Step 2 running:

1. Click **Open Connection**.
2. Choose **Foxglove WebSocket**.
3. Enter **`ws://localhost:9876`** and connect.

### Panels worth adding

![Lichtblick showing the Go2's LiDAR point cloud with the robot model at the
centre, plus camera, teleop, battery gauge, and sport-mode publish
panels](docs/lichtblick-view.png)

The layout above is what the panels below add up to: the **3D** panel filling the
top with the LiDAR point cloud and the dog itself, and a row of **Publish**,
**Image**, **Teleop**, and **Gauge** panels beneath it.

Add any panel with the **`+`** button (top-left), then set its topics via the
gear icon on the panel's top-right.

#### 3D — the dog and its LiDAR

The best visual in the demo: a live 3D model of the dog with the LiDAR point
cloud accumulating around it.

1. Add (`+`) → **3D** panel.
2. In panel settings, expand **Topics** and enable:
   - **`/robot_description`** — the dimensionally-correct model of the dog, which
     moves as its real joints move
   - **`/point_cloud2`** — the LiDAR returns
   - **`/scan`** — the 2D laser slice, optional but it reads well against the cloud
3. Under each of those two settings, set **Color mode** to `Color map`, **Color by**
   to `intensity`, and **Color map** to `Turbo`. Bump the point cloud's **Point
   size** to ~`5` so the returns stay visible while the dog moves.
4. Set **Frame → Follow mode** to **`Pose`** and **Display frame** to
   **`base_link`** to keep the camera locked on the dog as it walks.
5. Leave the image topics (`/camera/image_raw`, `/annotated_image/compressed`) and
   `/map` **off** here — the Image panel below handles the camera, and over the
   tunnel projecting it into the 3D scene just costs bandwidth.

Drag with the left mouse button to orbit, scroll to zoom.

#### Image — the camera

Add (`+`) → **Image** panel, then set **Topic** to
**`/annotated_image/compressed`** — the front camera with object-detection boxes
drawn on. Use `/camera/image_raw/compressed` for the clean feed.

**Use the `/compressed` topics over the tunnel.** Raw camera saturates the SSM
tunnel and lags badly.

#### Teleop — arrow-key driving

1. Add (`+`) → **Teleop** panel.
2. In panel settings set **Topic** to **`/cmd_vel_out`** (**not** the default
   `/cmd_vel` — the driver doesn't listen there).
3. Set the **Up / Down** speed to `0.6` and **Left / Right** to `1.0`.

Then click into the panel and hold an arrow key. **Hold it down** — the panel
publishes while a key is held, so tapping only twitches the dog.

> There is **no "Publish rate" setting** in the Teleop panel; the rate is fixed.
> (Older Foxglove docs mention one.) If the dog only twitches, you're tapping
> rather than holding, or it isn't in BalanceStand yet.

#### Battery

Add (`+`) → **Gauge** panel, set **Message path** to
**`/lowstate.bms_state.soc`** (state of charge, 0–100), with **Min** `0` and
**Max** `100`.

#### Publish — raw sport-mode commands

Add (`+`) → **Publish** panel, set **Topic** to `/webrtc_req` and **Schema** to
`go2_interfaces/msg/WebRtcReq`, then publish e.g. StandUp:

```json
{ "api_id": 1004, "parameter": "", "topic": "rt/api/sport/request" }
```

`1004` = StandUp, `1002` = BalanceStand (required before walking), `1005` =
StandDown, `1009` = Sit. The
[full command table](DEVELOPER_GUIDE.md#driving-the-robot) has all ~50.

### Watching the KVS video stream

Alongside Lichtblick, the container streams the **object-detection-annotated
camera feed** to **Amazon Kinesis Video Streams** — a smoother picture than the
SSM tunnel can carry, and the same stream the cloud vision commands read from.

In the AWS console (signed into the demo account):

**Kinesis Video Streams → Video streams → `go2-robot-01-camera` → Media playback**

Pick **Live** for the current feed, or scrub the timeline to replay up to 24 hours
of recorded video. Auth is your AWS login, so there's nothing to share to show it
on a screen.

Expect **a few seconds of latency** — KVS is an observation feed, not a control
link. Drive from Lichtblick, and use the KVS view for the wide shot.

---

## Clean up — stop the robot when done

Ctrl-C the `tunnel` (and `ap-bridge`, if running) **first**, then:

```bash
make close-ec2
make stop-ec2
```

Order matters: stopping the instance kills the tunnel's remote end but leaves the
local port bound, so a tunnel left open afterwards silently hangs the next client.

Nothing is lost — scene memory is per-session and KVS keeps its own buffer.
**Coordinate first**: someone else may still be using it.

**If everyone forgets, the instance stops itself 4 hours after it started** — a
watchdog Lambda in `Go2Ec2Stack` checks every 5 minutes, because a g4dn.xlarge left
running overnight costs more than a month of everything else here. `make status-ec2`
prints how long is left; `make extend-ec2` buys another hour when a session runs
long. See [AWS_DEPLOYMENT.md](AWS_DEPLOYMENT.md#daily-use) for the budget knobs.

> **Cost note:** a stopped instance costs only its EBS volume (**~$8/mo**), and
> that volume is what keeps the next start at ~40 seconds instead of a cold 14 GB
> image pull — so leave it stopped rather than tearing it down between sessions.
>
> The VPC's four interface endpoints (`ecr.api`, `ecr.dkr`, `logs`,
> `bedrock-runtime` — **~$29/mo** together) keep running regardless. They are what
> lets the hosted AgentCore agent pull its image and reach Nova Sonic from an
> isolated subnet with no internet route at all; the robot itself uses none of them.
> To stop everything, tear the stacks down in reverse order — see
> [AWS_DEPLOYMENT.md](AWS_DEPLOYMENT.md#gotchas).

---

## Troubleshooting

- **`[Errno 2] No such file or directory: 'isengardcli'`** — the AWS CLI can't
  exec the credential helper. The Makefile prepends `~/.toolbox/bin` to `PATH`
  itself, so this only bites when you run the scripts directly; add
  `export PATH="$HOME/.toolbox/bin:$PATH"` to `~/.zshenv`, which *every* zsh
  sources.
- **`Cannot detect shell`** from `isengardcli` — it reads `$SHELL`; set it if
  your environment strips it.
- **`make tunnel` exits saying the local port is in use** — something else holds
  9876. Find it with `lsof -nP -iTCP:9876 -sTCP:LISTEN`. Quit that process, or use
  another port: `make tunnel GO2_LOCAL_PORT=<port>` (then
  `make voice-cloud GO2_LOCAL_PORT=<port>` to match).
- **Long-running targets need their own terminal** — `tunnel`, `ap-bridge` and the
  agent each hold their terminal until you Ctrl-C them. Use a separate tab per
  target rather than backgrounding them; the agent in particular owns stdin
  (SPACE toggles the mic).
- **Lichtblick connects but shows no topics** — the bridge stays up but drops all
  channels when the driver dies, so "connected with nothing in it" almost always
  means the container is running but has no robot link. Run `make status-ec2`: it
  prints the instance state, the container state, the current robot-link mode, and
  the last of the unit log. `code=1000 "Device not online"` there means the dog
  itself is offline (powered off, no internet, or the app's Internet/remote toggle
  is off).
- **AP mode: `ap-bridge` connects to the dog, then dies with
  `ServerDisconnectedError` on `localhost:9876`** — the tunnel is fine; nothing is
  listening on the instance's 8765, so the remote end closes immediately. Run
  `make status-ec2`: if the unit is `failed` with a ROS parameter tuple error and
  `Robot IPs: []`, the image predates bridge support — `make push-image &&
  make restart-ec2`.
- **AP mode: *"what do you see?"* works but *"find a person"* does nothing** — the
  camera is fine; the detector is starved. Those two commands take different paths:
  the vision answer reads `/camera/image_raw/compressed` off the bridge and calls
  Bedrock itself, while *"find a person"*, *"follow me"* and the greeter all need
  `/detected_objects`, which requires `coco_detector_node` to be receiving frames.
  On a current image `CONN_TYPE=bridge` points it at the compressed topic
  automatically; on an older one it sits on raw `/camera/image_raw`, which nothing
  publishes in AP mode. `make push-image && make restart-ec2`. Same root cause if
  `/annotated_image/compressed` is missing in Lichtblick or the KVS stream is empty
  — both live downstream of the detector.
- **AP mode: topics stop but nothing reports an error** — EC2 can't distinguish a
  dead bridge from an idle robot, so check the `make ap-bridge` heartbeat. If it is
  gone or its counters are frozen, restart it; the instance needs nothing.
- **AP mode: `make ap-bridge` can't reach the dog** — run `make ap-probe`. The
  usual causes are the laptop not actually joined to the hotspot, or the dog asleep
  on low battery.
- **AP mode: the robot rejects the connection** — only one WebRTC peer at a time.
  Make sure the instance is in bridge mode (`make status-ec2` should report
  `unitree creds in container: no`) and close the Unitree mobile app.
- **Nothing happens when you tell it to walk** — the dog isn't in BalanceStand.
  Say *"stand up"*, then *"balance stand"*.
- **Agent starts but the robot never responds** — the container may be up
  without a robot link (dog asleep on low battery, or busy with another WebRTC
  client). Close the Unitree mobile app and check the battery.
- **The tunnel worked earlier and now hangs with no error** — it's stale. Stopping
  or restarting the instance kills the session's remote end while
  `session-manager-plugin` keeps holding port 9876. Kill it and re-run
  `make tunnel`.

More troubleshooting — local/Docker setup, WebRTC handshake failures, robot IP
drift — is in
[DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md#troubleshooting).

---

## How it works

The diagrams above are the two shapes; this is the reasoning behind them.

Your laptop runs the two clients — **Lichtblick** for telemetry and teleop, and the
**voice agent** for speech. Both talk only to `localhost:9876`, which an
**IAM-gated SSM tunnel** forwards to the instance's Foxglove bridge. There is no
public endpoint and the security group allows nothing inbound from the internet, so
access is controlled by who can start an SSM session, not by an open port.

The **hosted** agent (`make voice-agentcore`) skips the tunnel entirely: it runs on
Bedrock AgentCore inside the same VPC and dials the bridge's fixed private IP.

The camera feed also branches off to **KVS** — the smooth video you watch in the
console, and the archive the cloud vision commands (*"kvs describe scene"*, *"kvs
compare scene"*) read frames from. **Bedrock** does the scene understanding on both
routes.

What changes between the modes is only the robot link:

- **Cloud mode** (`make ec2-remote`) — the instance reads the Unitree email,
  password and serial from one Secrets Manager secret, exchanges them for a TURN
  token, and reaches the dog over WebRTC.
- **AP mode** (`make ec2-bridge`) — the instance holds no robot link at all. Its
  driver node still runs (so the command topics stay advertised) but never
  connects, and the credentials are **not fetched**, so it cannot reach Unitree's
  cloud even in principle. Your laptop's bridge owns the WebRTC session and
  publishes the robot's topics into EC2's ROS graph over the same tunnel, using
  `foxglove_bridge`'s client-publish support — which is the same mechanism the
  voice agent already uses to send `/cmd_vel_out`.

Because EC2 only ever sees ROS topics in AP mode, it cannot tell a bridge that has
stopped from a robot standing still. If topics go quiet, check the `ap-bridge`
heartbeat rather than the instance.

The whole point of the AP path is that the dog talks to nothing but your laptop:
no login, no token, no TURN server, and nothing for a WAF to refuse.

---

## Imagery, people and privacy

This system points a camera at a room, sends what it sees to a vision model, and
speaks a description out loud. In greeter mode it does that **automatically**, to
whoever walks up, with no one asking it to. If you run it anywhere people are
present, that is personal data processing and it is yours to justify.

**What is captured and where it goes**

| Data | Where it lives | How long |
| --- | --- | --- |
| Camera frames | In the agent's memory, up to `SCENE_MEMORY_MAX` snapshots (default 120) | The life of the agent process |
| Camera frames, on disk | Only if `SCENE_MEMORY_DIR` is set — off by default | `SCENE_MEMORY_TTL_SECONDS` (default 30 min), capped at `SCENE_MEMORY_DISK_MAX` files, swept after every write |
| Camera frames, in AWS | The Kinesis Video Stream, only if `KVS_ENABLED=true` / `kvs:=true` | **24 hours** (`dataRetentionInHours` in `Go2KVSStack`) |
| Text descriptions of people | Sent to Amazon Bedrock; spoken aloud; held in the agent's session memory | The life of the session |
| Transcribed speech | Sent to Amazon Bedrock (Nova Sonic). **Not** written to logs — see the note in `voice_agent/tools.py` | Not retained by this code |

**What is already done for you**

- The vision prompts explicitly forbid identification. `GREETER_VISITOR_PROMPT`
  asks for position and clothing colour only and forbids name, age, gender and
  personal characteristics; `SCENE_SYSTEM_PROMPT` and the Lambda's
  `SYSTEM_GUARDRAIL` say the same, and the latter cannot be overridden by a
  caller's question.
- The Bedrock Guardrail anonymises `NAME`, `EMAIL` and `PHONE` in model output.
- On-disk snapshots are written 0600 in a 0700 directory, with a TTL and a count
  cap enforced on every write.
- Tool logging is metadata only. What someone said is never written to CloudWatch.

**What is still yours to do**

- Establish a lawful basis for recording people, and give notice — a sign in the
  space is the usual minimum.
- Decide a retention period and set `SCENE_MEMORY_TTL_SECONDS`, the KVS
  `dataRetentionInHours`, and the CloudWatch log-group retention to match it.
- Leave `SCENE_MEMORY_DIR` unset unless you need it, and put it on encrypted
  storage if you do.
- Turn `KVS_ENABLED` off when you are not demonstrating the cloud video path.

## Cost and rate limits

Per-call token budgets are set everywhere (`SCENE_MAX_TOKENS`, the Lambda's
`max_tokens`), and `SCENE_MEMORY_MAX` bounds memory. **Nothing bounds spend across
a session.** A long-lived voice session, or a greeter left watching an empty
corridor, will keep invoking Bedrock for as long as it runs.

That is acceptable for a sample and is called out rather than solved. On a path to
production you would want a per-session token budget, a request rate limit, and a
CloudWatch billing alarm. The EC2 side already has an equivalent control worth
copying: the [4-hour auto-stop](AWS_DEPLOYMENT.md) watchdog exists because the
expensive failure mode there was a forgotten `make stop-ec2`, and the same logic
applies to a forgotten agent session.

---

## Documentation

| Doc | Contents |
| --- | --- |
| **[RESPONSIBLE-AI.md](RESPONSIBLE-AI.md)** | Intended and out-of-scope uses, how model output reaches the actuators, known failure modes, the guardrail configuration, and what human oversight this assumes |
| **[DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md)** | Local macOS/Docker setup, SLAM & Nav2, mapping, object detection, KVS camera streaming, full command reference, troubleshooting |
| **[AWS_DEPLOYMENT.md](AWS_DEPLOYMENT.md)** | Deploying and operating the cloud stacks — architecture and cost, one-time setup, daily `make` targets, the auto-stop budget, cloud troubleshooting |
| [voice_agent/README.md](voice_agent/README.md) | Voice agent internals, tools, configuration, and the [MuJoCo simulation](voice_agent/README.md#simulation-main_simpy) |
| [deployment/README.md](deployment/README.md) | CDK stacks — VPC, robot EC2, AgentCore runtime, KVS stream |
| [Run a ROS 2 robot stack on Amazon EC2 and drive it with a voice agent](https://builder.aws.com/content/3ISz7ce0hvmQ8i4d9BIhzntb6fQ/run-a-ros-2-robot-stack-on-amazon-ec2-and-drive-it-with-a-voice-agent) | The write-up behind this demo — why the stack is shaped this way |
| [Demo video](https://www.youtube.com/watch?v=Zu9zC-Oa7qM) | The robot driven by voice, end to end |

## Credits

Built on [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) by
[@abizovnuralem](https://github.com/abizovnuralem), itself based on
[@tfoldi](https://github.com/tfoldi)'s [go2-webrtc](https://github.com/tfoldi/go2-webrtc).
Full acknowledgements in [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md#thanks).

## License

MIT-0 for Amazon's contributions — see [LICENSE](LICENSE). This repository also
redistributes upstream code under BSD-2-Clause, BSD-3-Clause, Apache-2.0, and MIT;
those notices must be retained and are reproduced in
[THIRD-PARTY-LICENSES](THIRD-PARTY-LICENSES).

## Trademarks

Unitree and Go2 are trademarks of Unitree Robotics. This project is not
affiliated with, sponsored by, or endorsed by Unitree Robotics or the RoboVerse
community. All other trademarks are the property of their respective owners.
