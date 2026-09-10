# Go2 ROS2 SDK — Developer Guide

> ⚠️ **Sample code — not for production use.** This repository is a demonstration
> sample with no AWS SLA or support. Read the
> [non-production disclaimer](README.md#%EF%B8%8F-sample-code--not-for-production-use)
> and [RESPONSIBLE-AI.md](RESPONSIBLE-AI.md) before adapting any of it.

Everything beyond the cloud voice demo: running the SDK locally, SLAM and
navigation, the KVS cloud pipeline, and the full command reference.

**Looking for the demo?** Start at the [README](README.md) — it covers account
access, the 3-step run, Lichtblick setup, and cleanup.

**Deploying or operating the cloud stacks?** That's
[AWS_DEPLOYMENT.md](AWS_DEPLOYMENT.md) — CDK stacks, one-time setup, the daily
`make` targets, and cloud troubleshooting.

## Contents

- [About the project](#about-the-project)
- [System requirements](#system-requirements)
- [Running via Docker (Linux)](#running-via-docker-linux)
- [Running on macOS (Docker)](#running-on-macos-docker)
- [Driving the robot](#driving-the-robot)
- [Troubleshooting](#troubleshooting)
- [Usage (native ROS2)](#usage-native-ros2)
- [SLAM and Nav2](#slam-and-nav2)
- [Real time image detection and tracking](#real-time-image-detection-and-tracking)
- [Voice Control Demo (Nova Sonic)](#voice-control-demo-nova-sonic)
- ["What do you see?" — Bedrock scene description](#what-do-you-see--bedrock-scene-description)
- ["What did you see earlier?" — robot memory](#what-did-you-see-earlier--robot-memory-temporal-qa)
- [KVS Camera Streaming](#kvs-camera-streaming)
- [Run the container in AWS](AWS_DEPLOYMENT.md)
- [3D raw pointcloud dump](#3d-raw-pointcloud-dump)
- [Multi robot support](#multi-robot-support)
- [WebRTC Topic Interface](#webrtc-topic-interface)
- [WSL 2](#wsl-2)
- [Thanks](#thanks)

---

## About the project

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
[![License](https://img.shields.io/badge/license-MIT--0-yellow.svg)](LICENSE)

This is an integration of the Unitree Go2 with ROS2 over Wi-Fi, designed by
[@tfoldi](https://github.com/tfoldi) — see his groundbreaking work at
[go2-webrtc](https://github.com/tfoldi/go2-webrtc).

This repo empowers Unitree GO2 AIR/PRO/EDU robots with ROS2 capabilities, using
both WebRTC (Wi-Fi) and CycloneDDS (Ethernet) protocols.

If you are using the WebRTC (Wi-Fi) protocol, close the connection with the
mobile app before connecting to the robot.

### Features

- Full ROS2 SDK support for your Unitree GO2
- Compatible with AIR, PRO, and EDU variants
- Access to foot force sensor feedback (some GO2 PRO models, and EDU)

### Project roadmap

1. URDF :white_check_mark:
2. Joint states sync in real time :white_check_mark:
3. IMU sync in real time :white_check_mark:
4. Joystick control in real time :white_check_mark:
5. Go2 topics info in real time :white_check_mark:
6. Foot force sensors info in real time :white_check_mark:
7. Lidar stream (added pointCloud2) :white_check_mark:
8. Camera stream :white_check_mark:
9. Foxglove WebSocket bridge (Lichtblick / OSS clients) :white_check_mark:
10. Laser Scan :white_check_mark:
11. Multi robot support :white_check_mark:
12. WebRTC and CycloneDDS support :white_check_mark:
13. Creating a PointCloud map and store it :white_check_mark:
14. SLAM (slam_toolbox) :white_check_mark:
15. Navigation (nav2) :white_check_mark:
16. Object detection (coco) :white_check_mark:
17. AutoPilot

### Real time Go2 Air/PRO/EDU joints sync

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/bf3f5a83-f02b-4c78-a7a1-b379ce057492" alt='Go2 joints sync'>
</p>

### Go2 Air/PRO/EDU lidar point cloud

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/9c1c3826-f875-4da1-a650-747044e748e1" alt='Go2 point cloud'>
</p>

---

## System requirements

Tested systems and ROS2 distro

| systems | ROS2 distro |
|--|--|
| Ubuntu 22.04 | iron |
| Ubuntu 22.04 | humble |
| Ubuntu 22.04 | rolling |

> The build-status column was dropped along with `.github/`: all three badges
> pointed at **upstream's** GitHub Actions run, not this fork's, so they reported
> a CI that never covered this code.

## Running via Docker (Linux)

Can set environment variables beforehand, hardcoded in docker/docker-compose.yaml, or as shown below.

Run:
```shell
cd docker
ROBOT_IP=<ROBOT_IP> CONN_TYPE=<webrtc/cyclonedds> docker-compose up --build
```

## Running on macOS (Docker)

The default Linux setup doesn't work on macOS as-is — `apt`, `network_mode: host`,
`/dev/input` joystick passthrough, and X11 forwarding all assume Linux. This
section covers running the SDK in a [Docker](https://www.docker.com/) container
on macOS using the Mac-compatible compose file at
[`docker/docker-compose.mac.yml`](docker/docker-compose.mac.yml).

### Prerequisites

| Item | Notes |
| --- | --- |
| Mac with Docker installed | `brew install --cask docker`, then start Docker Desktop |
| Unitree Go2 (AIR/PRO/EDU) on firmware ≥1.1.15 | The `data2 == 3` LAN auth path is required and supported here |
| Unitree mobile-app account (email + password) | Used **once** to fetch the per-device AES-128 key |
| Robot's serial number | Visible in the mobile app under Device → Info → SN |
| Robot's IP | Mobile app → Device → Data → Automatic Machine Inspection → STA Network: wlan0 |
| The Mac and robot on the same network | Corporate WiFi with **client isolation** silently blocks all traffic — see Troubleshooting below |

### Get the AES-128 key

Firmware ≥1.1.15 wraps the LAN handshake's RSA public key with a per-device
AES-128 key that the robot only releases through Unitree's cloud. Without it,
the WebRTC handshake fails with `RSA key format is not supported`.

Fetch it once from Unitree's cloud. You'll need the Unitree mobile-app
email + password the robot is registered to:

```bash
# In a Mac shell — uses your Unitree login, NOT the SDK's container.
make venv
.venv/bin/pip install 'unitree_webrtc_connect==2.2.0'
.venv/bin/unitree-fetch-aes-key \
    --email YOUR@EMAIL --password 'YOURPASS' --device-type Go2
```

Copy the 32-character hex key from the output — it goes in `docker/.env`
(next section). Add `--region cn` if the account is registered in China.
The key is per-device and does not change, so fetch it once per robot; if
you keep a fleet, storing it in AWS Secrets Manager saves the next person
the round trip.

### Configure

Create `docker/.env` (overwrites whatever is there):

```ini
ROBOT_IP=172.24.1.87          # IP of your robot from the mobile app
AES_128_KEY=<32 hex chars>    # from unitree-fetch-aes-key
CONN_TYPE=webrtc
WEBRTC_SERVER_PORT=9991
```

> The robot's IP changes between networks. `172.24.1.87` is the current
> address on this lab's WiFi — verify with the Unitree mobile app each
> time you switch networks.

Sanity-check that your Mac can reach the robot:

```bash
nc -z -v -G 5 $ROBOT_IP 9991
# expect: "Connection to <ip> port 9991 [tcp/osm-oev] succeeded!"
```

If that times out, your Mac and the robot are not on the same reachable
network. Common causes: corporate WiFi with client isolation; you're on
home WiFi but the robot is on its own hotspot; battery is too low and the
robot is in standby.

### Build and run

> All `docker` commands in this section work identically with any
> docker-compatible CLI — just substitute the binary
> (`docker compose …`, `docker exec …`, etc.).

```bash
cd docker
docker compose -f docker-compose.mac.yml --env-file .env up -d --build
```

The first build takes ~30 minutes (downloads ROS Humble base image, installs
Nav2 / SLAM / foxglove_bridge, builds the workspace via colcon). Subsequent runs
use the cache and start in seconds.

Watch the driver come up:

```bash
tail -f docker/logs/robot.log | grep -E "validated|Failed|Successfully"
```

You're ready when you see `Robot 0 validated and ready`.

### Restart (reconnect to the dog)

If the WebRTC link gets stuck (stale peer, battery dipped into standby, robot
busy with another client), restart the container:

```bash
cd docker
docker compose -f docker-compose.mac.yml --env-file .env restart
```

This kills the driver, drops the WebRTC session cleanly, and brings everything
back up — usually faster than waiting for the SDK's auto-retry. Watch
`docker/logs/robot.log` again for `Robot 0 validated and ready`.

### Connect Lichtblick (OSS viewer)

When running **locally**, the bridge is published on port **8766** on your Mac
(8765 is reserved because something else commonly listens on it). Open
Lichtblick in your browser — no install required — at:

https://lichtblick-suite.github.io/lichtblick/

Then:

1. Click **Open Connection**.
2. Choose **Foxglove WebSocket**.
3. Enter:

```
ws://localhost:8766
```

> Running the **cloud** demo instead? The SSM tunnel uses port **9876** — see
> the [README](README.md#watch-it-in-lichtblick).

### Setting up Lichtblick panels

Once connected, build a layout with the panels below. Add any panel with
the **`+`** button (top-left), then set its topic/settings via the gear
icon on the panel's top-right.

#### Camera

1. Add (`+`) → **Image** panel.
2. In panel settings, set **Topic** to **`/annotated_image`** — this is the
   front camera with COCO object-detection bounding boxes overlaid (published
   by `coco_detector_node`, which runs automatically with the SDK).
   Use **`/camera/image_raw`** if you want the clean feed without annotations.

#### Teleop (arrow-key driving)

1. Add (`+`) → **Teleop** panel.
2. In panel settings, change **Topic** from the default `/cmd_vel` to
   **`/cmd_vel_out`** (the driver subscribes here, not `/cmd_vel`).
3. Set **Publish rate** to ~`10 Hz` — one-shot publishes only twitch the
   dog; it needs a continuous stream.
4. Sensible limits: **Max linear** `0.6` m/s, **Max angular** `1.0` rad/s.

Arrow keys do nothing until step 2 is done **and** the dog is in
BalanceStand (see below).

#### Battery indicator

1. Add (`+`) → **Gauge** (or **Indicator**) panel.
2. Set **Message path** to **`/lowstate.bms_state.soc`** (state of
   charge, 0–100 %).
3. On a Gauge, set **Min** `0`, **Max** `100`.

#### `/webrtc_req` publish panel (sport-mode commands)

This panel sends Unitree sport-mode commands — including the stand
sequence that makes the dog move.

1. Add (`+`) → **Publish** panel.
2. Set **Topic** to **`/webrtc_req`**.
3. Set **Message schema** to **`go2_interfaces/msg/WebRtcReq`**.
4. Paste this JSON and click **Publish**:

```json
{ "api_id": 1004, "parameter": "", "topic": "rt/api/sport/request" }
```

### Making the dog move (stand sequence)

Send these two commands from the `/webrtc_req` **Publish** panel, in
order. **Both are required before the dog will respond to teleop.**

1. **StandUp** — joints lock, dog rises rigidly:

```json
{
  "api_id": 1004,
  "parameter": "",
  "topic": "rt/api/sport/request"
}
```

2. **BalanceStand** — engages active balance; this is the mode that
   actually walks. **Without it the dog ignores `/cmd_vel_out`:**

```json
{
  "api_id": 1002,
  "parameter": "",
  "topic": "rt/api/sport/request"
}
```

Then drive from the **Teleop** panel (arrow keys) or republish
`/cmd_vel_out`. To shut down, send **StandDown** (`1005`) or **Sit**
(`1009`). Both stand commands are idempotent — safe to resend if the dog
isn't responding.

## Driving the robot

> **Pre-flight checklist — do this every time you boot the dog**
>
> 1. **Stand the dog up** with `api_id: 1004` on `/webrtc_req`. Joints
>    lock, the dog rises rigidly. Velocity commands are ignored in this
>    state — it just stays put.
> 2. **Engage active balance** with `api_id: 1002` (BalanceStand). This
>    is the mode that actually walks. **Without 1002 the dog will not
>    move from `/cmd_vel_out` no matter what you publish.**
> 3. Drive with `/cmd_vel_out` (Lichtblick Teleop panel, ROS CLI, etc.).
>
> The two stand commands are idempotent — safe to send again if the
> dog isn't responding. To shut down: `api_id: 1005` (StandDown) or
> `1009` (Sit).

```bash
# 1. Stand up (joints lock, dog rises rigidly)
docker exec docker-unitree_ros-1 bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq '{api_id: 1004, parameter: \"\", topic: \"rt/api/sport/request\"}'"

# 2. Engage balance stand (REQUIRED before walking)
docker exec docker-unitree_ros-1 bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq '{api_id: 1002, parameter: \"\", topic: \"rt/api/sport/request\"}'"

# 3. Walk forward at 0.3 m/s for as long as you keep republishing
docker exec docker-unitree_ros-1 bash -lc "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.0}}'"
```

#### Lichtblick Teleop panel setup

1. Add (`+`) → **Teleop** panel.
2. Open panel settings (gear icon on the panel's top-right).
3. Change **Topic** from the default `/cmd_vel` to **`/cmd_vel_out`**.
4. Set **Publish rate** to ~`10 Hz` (one-shot publishes from
   `/cmd_vel_out` only twitch the dog; it needs a continuous stream).
5. Sensible limits: **Max linear** `0.6` m/s, **Max angular** `1.0` rad/s.

Without step 3 (the topic change), arrow keys do nothing because the
driver isn't listening on `/cmd_vel`.

The driver subscribes to **`/cmd_vel_out`**, not `/cmd_vel` (the launch
file expects a velocity smoother in front of the driver; with Nav2
disabled there's no smoother, so publish directly to the post-smoother
topic). If you re-enable Nav2, switch back to `/cmd_vel`.

Unitree sport-mode commands (publish to `/webrtc_req` with
`topic: rt/api/sport/request`). Full list mirrored from the
[RoboVerse wiki](https://wiki.theroboverse.com/en/unitree-go2-app-console-commands):

| api_id | Action | Notes |
| --- | --- | --- |
| 1001 | Damp | joints relax — dog flops down |
| 1002 | BalanceStand | **required before walking** |
| 1003 | StopMove | |
| 1004 | StandUp | rigid stand |
| 1005 | StandDown | |
| 1006 | RecoveryStand | get up after falling |
| 1007 | Euler | |
| 1008 | Move | what `/cmd_vel_out` ends up calling |
| 1009 | Sit | cross-legged sit |
| 1010 | RiseSit | |
| 1011 | SwitchGait | |
| 1012 | Trigger | |
| 1013 | BodyHeight | takes parameter, e.g. `'{"data":-0.18}'` (range ≈ -0.18 lowest to +0.03 highest) — use this for "crouch" |
| 1014 | FootRaiseHeight | |
| 1015 | SpeedLevel | |
| 1016 | Hello | paw wave |
| 1017 | Stretch | |
| 1018 | TrajectoryFollow | |
| 1019 | ContinuousGait | |
| 1020 | Content | not implemented on the robot |
| 1021 | Wallow | |
| 1022 | Dance1 | |
| 1023 | Dance2 | |
| 1024 | GetBodyHeight | not implemented on the robot |
| 1025 | GetFootRaiseHeight | not implemented on the robot |
| 1026 | GetSpeedLevel | not implemented on the robot |
| 1027 | SwitchJoystick | broken — use bash_runner_client instead |
| 1028 | Pose | |
| 1029 | Scrape | |
| 1030 | FrontFlip | ⚠️ needs space and full battery |
| 1031 | FrontJump | |
| 1032 | FrontPounce | |
| 1033 | WiggleHips | |
| 1034 | GetState | |
| 1035 | EconomicGait | |
| 1036 | FingerHeart | |
| 1042 | LeftFlip | |
| 1043 | RightFlip | |
| 1044 | Backflip | ⚠️ needs space and full battery |
| 1045 | LeadFollow | |
| 1301 | Handstand | ⚠️ |
| 1302 | CrossStep | |
| 1303 | OnesidedStep | |
| 1304 | Bound | running gait |

### Checking battery level

Battery state is published on `/lowstate` (`lowstate.bms_state.soc`).

## Troubleshooting

- **Robot IP keeps changing** — the Go2's WiFi IP drifts frequently.
  The Unitree mobile app's IP field lags and often shows a stale value.
  Find the real current IP by MAC address:
  ```bash
  arp -a | grep -i "78:22:88"
  ```
  If nothing shows, ping the last-known IP first to populate the cache,
  then retry. If the robot has moved to a different subnet, do a full
  sweep (takes ~2 min on a /17):
  ```bash
  for sub in $(seq 0 127); do
    for i in $(seq 1 254); do ping -c1 -W1 172.24.$sub.$i >/dev/null 2>&1 & done
    wait
  done
  arp -a | grep -i "78:22:88"
  ```
  Verify the result is actually the dog (not a random device):
  ```bash
  curl -s --max-time 4 http://<ip>:9991/con_notify -X POST -d '{}' \
    | python3 -c "import sys,base64,json; d=base64.b64decode(sys.stdin.read()); print(json.loads(d).get('data2'))"
  # should print: 3
  ```
  Update `docker/.env` with the new IP, then do a full recreate (see
  next bullet for why `restart` is not enough).

- **`AES_128_KEY` missing / driver exits immediately with key error** —
  the `docker compose` commands **must be run from the `docker/`
  directory** (or with `--env-file docker/.env`) so that `.env` is
  picked up and all variables (including `AES_128_KEY`) are injected.
  Running compose from the repo root causes `AES_128_KEY` to be empty
  in the container, and the driver exits immediately with:
  `This robot speaks data2=3 — the per-device AES-128 key is required`.
  Also, **`docker compose restart` does NOT re-read `.env`** — after any
  `.env` change (including a new `ROBOT_IP`) you must do a full recreate:
  ```bash
  cd docker
  docker compose -f docker-compose.mac.yml down
  docker compose -f docker-compose.mac.yml up -d
  ```

- **Lichtblick connects but shows no topics / port 8766 refuses** —
  the foxglove bridge process stays up but removes all channel
  advertisements when `go2_driver_node` crashes. Check the driver logs
  first:
  ```bash
  docker compose -f docker/docker-compose.mac.yml logs --tail=30 unitree_ros \
    | grep -iE "error|failed|closed|validated"
  ```
  If you see `AES_128_KEY` errors or `process has finished cleanly`,
  the driver died — fix the root cause (usually missing key or wrong
  IP) and do a full `down && up -d` from the `docker/` directory.

- **`peer=connecting, ice=completed` then timeout** — DTLS handshake
  stalled. The robot probably has a stale peer from a prior attempt;
  wait 15-30 s and let the SDK auto-retry, or `docker compose ...
  restart` once.
- **`RSA key format is not supported`** — the AES-128 key is missing
  or wrong; re-run `unitree-fetch-aes-key`.
- **`Robot signaling returned no SDP answer` / `Connection aborted`**
  — the robot is busy with another WebRTC client. Close the Unitree
  mobile app, kill any other script that connected, wait 10-20 s.
- **`Connection to <ip>:9991 timed out`** — the robot is unreachable.
  Either it's asleep (battery low → standby), powered off, or your
  WiFi is blocking client-to-client traffic. Probe with `nc -z` from
  the Mac first.
- **VM disk full during build** — Docker's VM has a fixed virtual disk
  size. Run `docker image prune -a -f` and `docker builder prune -f` to
  free space (or raise the limit in Docker Desktop → Settings →
  Resources), then retry the build.
- **Lichtblick "WebSocket server at ws://localhost:8765 not reachable"**
  — port 8765 is taken by something else on your Mac (we publish on
  8766 instead). Connect to `ws://localhost:8766`. If you also need
  8766 for something, edit the port mapping in
  `docker/docker-compose.mac.yml`.
- **Port 8766 already in use / bridge won't bind** — Amazon Quick may
  be competing for port 8766. Quit Amazon Quick (or whatever is holding
  the port — check with `lsof -i :8766`) and restart the container.
- **Battery dies mid-test** — the WiFi link drops as the robot enters
  standby, the driver hangs trying to reconnect. Plug in the charger,
  wait for the WiFi link to come back, then `restart` the container.

### What the Mac compose file changes vs. Linux

- `network_mode: bridge` instead of `host` (no host networking on
  macOS — Docker's VM is in the way).
- Explicit `9991:9991/tcp+udp` and `8766:8765/tcp` port forwarding.
- No `/dev/input` joystick passthrough (no joysticks on macOS).
- No X11 mounts (no XQuartz dependency; use Lichtblick instead of RViz).
- Logs tee'd to `docker/logs/robot.log` on the host so you can `tail`
  and `grep` them with normal Mac tools instead of `docker logs`.

### What's enabled by default

The Mac launch command in `docker-compose.mac.yml` only starts the
driver, foxglove_bridge WebSocket server, and camera:

| Feature | State | How to flip |
| --- | --- | --- |
| Driver (`go2_driver_node`) | ✅ on | always on |
| WebSocket bridge for Lichtblick (port 8766) | ✅ on | `foxglove:=true` |
| Front camera (`/camera/image_raw`) | ✅ on | parameter `enable_video` (default `True` in [`go2_driver_node.py`](go2_robot_sdk/go2_robot_sdk/presentation/go2_driver_node.py)) |
| LiDAR (`/point_cloud2`) | ✅ on | parameter `decode_lidar` (default `True`) |
| KVS streaming (`kvs_producer_node`) | ❌ off | set `KVS_ENABLED=true` in `docker/.env` |
| RViz2 | ❌ off | `rviz2:=true` (needs XQuartz on macOS — flaky) |
| Nav2 | ❌ off | `nav2:=true` |
| SLAM (`slam_toolbox`) | ❌ off | `slam:=true` |
| Joystick driver | ❌ off | `joystick:=true` (no joystick on macOS — pointless) |
| Teleop (twist_mux) | ❌ off | `teleop:=true` (companion to joystick) |

To re-enable any of these, edit the launch flags in
[`docker/docker-compose.mac.yml`](docker/docker-compose.mac.yml) and
restart the container. Nav2 + SLAM + RViz pull in significant CPU /
memory; expect 1-2 GiB of extra image size and slower startup.

> Note: `/cmd_vel_out` is published by the velocity smoother (part of
> Nav2). With Nav2 disabled, no smoother runs, and you publish
> directly to `/cmd_vel_out` from Lichtblick or the CLI as shown above.
> If you re-enable Nav2, switch back to publishing on `/cmd_vel`.

## Usage (native ROS2)

Don't forget to set up your Go2 robot in Wifi-mode and obtain the IP. You can use the mobile app to get it. Go to Device -> Data -> Automatic Machine Inspection and look for STA Network: wlan0.

```shell
source install/setup.bash
export ROBOT_IP="robot_ip" #for muliple robots, just split by ,
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py
```

The `robot.launch.py` code starts many services/nodes simultaneously, including

* robot_state_publisher
* ros2_go2_video (front color camera)
* pointcloud_to_laserscan_node
* go2_robot_sdk/go2_driver_node
* lidar_processor/lidar_to_pointcloud
* rviz2
* `joy` (ROS2 Driver for Generic Joysticks and Game Controllers)
* `teleop_twist_joy` (facility for tele-operating Twist-based ROS2 robots with a standard joystick. Converts joy messages to velocity commands)
* `twist_mux` (twist_multiplexer with source prioritization)
* foxglove_launch (launches the foxglove bridge)
* slam_toolbox/online_async_launch.py
* nav2_bringup/navigation_launch.py

When you run `robot.launch.py`, `rviz` will fire up, lidar data will begin to accumulate, the front color camera data will be displayed too (typically after 4 seconds), and your dog will be waiting for commands from your joystick (e.g. a X-box controller). You can then steer the dog through your house, e.g., and collect LIDAR mapping data.

## SLAM and Nav2

![Simplified Rviz Display](https://github.com/user-attachments/assets/74a7c07c-2c2d-4022-9a23-94407f2c2a06)

The goal of SLAM overall, and the `slam_toolbox` in particular, is to create a map. The `slam_toolbox` is a grid mapper - it thinks about the world in terms of a fixed grid that the dog operates in. When the dog initially moves through a new space, data accumulate and the developing map is and published it to the `/map` topic. The goal of `Nav2` is to navigate and perform other tasks in this map.

The `rviz` settings that are used upon initial launch (triggered by `ros2 launch go2_robot_sdk robot.launch.py`) showcase various datastreams.

* `RobotModel` is the dimensionally correct model of the G02
* `PointCloud2` are the raw LIDAR data transformed into 3D objects/constraints
* `LaserScan` are lower level scan data before translation into an x,y,z frame
* `Image` are the data from the front-facing color camera
* `Map` is the map being created by the `slam_toolbox`
* `Odometry` is the history of directions/movements of the dog

If there is too much going on in the initial screen, deselect the `map` topic to allow you to see more.

### Mapping - creating your first map

Use painter's tape to mark a 'dock' rectangle (or use a real dock) to create a defined starting point for your dog on your floor. In the `rviz` `SlamToolboxPlugin`, on the left side of the your `rviz` screen, select "Start At Dock". Then, use your controller to manually explore a space, such as a series of rooms. You will see the map data accumulating in `rviz`. In this map, white, black and grey pixels represent the free, occupied, and unknown space, respectively. When you are done mapping, enter a file name into the "Save Map" field and click "Save Map". Then enter a file name into "Serialize Map" field and click "Serialize Map". Now, you should have 2 new files in `/ros2_ws`:

```shell
map_1.yaml: the metadata for the map as well as the path to the .pgm image file.
map_1.pgm: the image file with white, black and grey pixels representing the free, occupied, and unknown space.
map_1.data: 
map_1.posegraph: 
```

The next time you start the system, the map can be loaded and is ready for you to complete/extend by mapping more spaces. Upon restart and loading a map, the dog does not know where it is relative to the map you created earlier. Assuming you rebooted the dog in its marked rectangle, or in an actual dock, it will have a high quality initial position and angle.

### Autonomous Navigation - navigating in your new map

As shown in the `rviz` `Navigation 2` plugin, the system will come up in:

```shell
Navigation: active
Localization: inactive
Feedback: unknown
```

Then, load your map via the `SlamToolboxPlugin` (enter your map's filename (without any extension) in the 'Deserialize Map' field and then click 'Deserialize Map').

**WARNING**: please make sure that (1) the dog is correctly oriented WRT to the map and (2) the map itself is sane and corresponds to your house. Especially if you have long corridors, the overall map can be distorted relative to reality, and this means that the route planner will try to route your dog through walls, leaving long scratches in your walls.

You can now give the dog its first target, via 'Nav2 Goal' in the `rviz` menu. Use the mouse cursor to provide a target to navigate to.

**NOTE**: the `Nav2 Goal` cursor sets both the target position and the final angle of the dog, that you wish the dog to adopt upon reaching the target (need to double check). The long green arrow that is revealed when you click an point and keep moving your mouse cursor is the angle setter.

Until you have some experience, we suggest following your dog and picking it up when it is about to do something silly.

**NOTE**: Virtually all fault behaviors - spinning in circles, running into walls, trying to walk through walls, etc reflect (1) a map that is incorrect, (2) incorrect initial position/angle of the dog relative to that map, or (3) inability to compute solutions/paths based on overloaded control loops. To prevent #3, which results in no motion or continuous spinning, the key loop rates (`controller_frequency`: 3.0 and `expected_planner_frequency`: 1.0 have been set to very conservative rates).

## Real time image detection and tracking

This capability is directly based on [J. Francis's work](https://github.com/jfrancis71/ros2_coco_detector). Launch the `go2_ros2_sdk`. After a few seconds, the color image data will be available at `/camera/image_raw`. Then start the detector in the running container:

```bash
docker exec -d docker-unitree_ros-1 bash -c \
  "source /ros_entrypoint.sh && ros2 run coco_detector coco_detector_node \
   --ros-args -p detection_threshold:=0.5"
```

There will be a short delay the first time the node is run for PyTorch TorchVision to download the neural network. You should see a download progress bar. TorchVision cached for subsequent runs.

To view the detection messages from the host:

```bash
docker exec docker-unitree_ros-1 bash -lc \
  "source /ros_entrypoint.sh && ros2 topic echo /detected_objects"
```

The detection messages contain the detected object (`class_id`) and the `score`, a number from 0 to 1. For example: `detections:results:hypothesis:class_id: giraffe` and `detections:results:hypothesis:score: 0.9989`. The `bbox:center:x` and `bbox:center:y` contain the centroid of the object in pixels. These data can be used to implement real-time object following for animals and people. People are detected as `detections:results:hypothesis:class_id: person`.

To view the annotated image stream, open Lichtblick at `ws://localhost:8766` and set the Image panel topic to **`/annotated_image`** (bounding boxes overlaid) instead of `/camera/image_raw`.

To adjust detection sensitivity or disable the annotated image:

```bash
docker exec -d docker-unitree_ros-1 bash -c \
  "source /ros_entrypoint.sh && ros2 run coco_detector coco_detector_node \
   --ros-args -p publish_annotated_image:=False -p device:=cpu -p detection_threshold:=0.7"
```

`detection_threshold` is between 0.0 and 1.0 — higher values reject more detections (default 0.5). Set `publish_annotated_image:=False` if you only need `/detected_objects` and not the overlaid image.

### Choosing the detector model

The node runs a torchvision Faster R-CNN on CPU. Pick a backbone with the
`model` parameter (or `coco_model:=` on `robot.launch.py`):

| `model` | Backbone | COCO mAP | 1 vCPU latency | Use when |
|---|---|---|---|---|
| `fast` | MobileNetV3-320 | 22.8 | ~60 ms | CPU-starved; only really finds people and large furniture |
| `balanced` **(default)** | ResNet50-FPN @480 | 37.0 | ~380 ms | Default. ~2.6 fps, clears the node's 2 fps throttle |
| `accurate` | ResNet50-FPN @800 | 37.0 | ~890 ms | Best small/distant-object recall; ~1.1 fps |
| `best` | ResNet50-FPN **v2** | 46.7 | ~3.9 s | Highest accuracy, but ~0.25 fps — too slow for greeter/tracking. Offline or GPU only |

`balanced` replaced `fast` as the default because MobileNet-320 was the weakest
detector in torchvision: it missed most non-person classes and misidentified
obvious ones (it labelled a dog a "cat" with 0.96 confidence). Note that `best`
is head-bound rather than resolution-bound, so lowering its input size does not
make it usable in the live loop.

```bash
# e.g. maximum recall for a stationary inspection, at ~1 fps
docker exec -d docker-unitree_ros-1 bash -c \
  "source /ros_entrypoint.sh && ros2 run coco_detector coco_detector_node \
   --ros-args -p model:=accurate -p detection_threshold:=0.4"
```

The annotated image draws the confidence next to each class name, so you can
tune `detection_threshold` by watching `/annotated_image` directly.

## Voice Control Demo (Nova Sonic)

Drive the dog with natural language — by voice or by typing. An Amazon Nova
Sonic speech-to-speech agent (built on [Strands Agents](https://github.com/strands-agents/sdk-python))
interprets commands like *"stand up"*, *"walk forward for two seconds"*, or
*"do a dance"* and routes them to the robot.

It runs on the **host** (not in the container) and needs **Python 3.12+**. One
venv, `.venv`, covers every host-side entry point — the SDK's container deps are
installed in the image and never touch it:

```bash
make venv
```

There are three interchangeable entry points (same agent + UI, different
transport) — plus a fourth that runs the agent in AWS, see
[voice_agent/README.md](voice_agent/README.md):

**ROS2 transport** (recommended — SDK container running, no Unitree credentials needed):

```bash
# 1. Ensure AWS credentials are set (env vars or ~/.aws/credentials):
export AWS_ACCESS_KEY_ID=...
export AWS_SECRET_ACCESS_KEY=...
export AWS_DEFAULT_REGION=us-east-1
aws sts get-caller-identity     # sanity check

# 2. Launch (voice + text):
.venv/bin/python -m voice_agent.main

# Or text-only (no mic):
.venv/bin/python -m voice_agent.main --text-only
```

Press **SPACE** on an empty prompt to start the mic; press **SPACE** again to
stop and send. Typed lines work too.

**Simulation** (no robot and no container — a MuJoCo Go2 in the same process):

```bash
make voice-sim          # voice + text, with a viewer window
make voice-sim-text     # text only
make sim-demo           # scripted stand/walk/sit/wave — no agent, no AWS at all
```

**Direct WebRTC** (no container — drives the robot directly, needs Unitree credentials):

```bash
.venv/bin/pip install 'unitree-webrtc-connect==2.2.0'
.venv/bin/python -m voice_agent.main_sonic
```

The **ROS2** entry point (`main.py`) needs **no Unitree credentials**: it
connects only to the `foxglove_bridge` this SDK's container exposes on
`ws://localhost:8766`, and the container owns the robot link (LAN + the AES-128
key in `docker/.env`). The **WebRTC** entry point (`main_sonic.py`) connects to
the robot itself, so it needs the Unitree account or AES-128 key. The
**simulation** entry point (`main_sim.py`) needs neither — nor a robot, a
container or a network.

> The simulation is the fastest way to work on the agent, the prompt or a tool
> without the dog: the same agent, prompt and tools drive a
> [Strands Labs robots](https://strandsagents.com/docs/labs/robots/) Go2 in
> MuJoCo, with a head camera `describe_scene` reads. Poses and balance are
> physically simulated; the walking gait is animated, and the detector- and
> Nav2-backed tools report that they are absent rather than pretending. On macOS
> the viewer window needs `mjpython` (the make targets use it). Full details:
> [voice_agent/README.md](voice_agent/README.md#simulation-main_simpy).

See [voice_agent/README.md](voice_agent/README.md) for the full walkthrough,
configuration (including the WebRTC `UNITREE_*` variables and the simulation's
`SIM_*` ones), and how each transport maps commands onto the robot.

## "What do you see?" — Bedrock scene description

Ask the robot what it sees and it answers out loud. This runs **entirely
host-side in the voice agent** — no container node, no separate TTS service. The
`describe_scene` tool grabs the latest `/camera/image_raw` frame off the Foxglove
bridge, sends it to **Amazon Bedrock** (Claude Sonnet 4.6, multimodal, via boto3)
with the visitor's question, and returns the description text so **Nova Sonic
speaks it in its own voice**.

Just say *"what do you see?"* (or *"what's on the table?"*) to the voice agent.
Because it reuses the same AWS credentials already required for Nova Sonic, there
is nothing extra to launch or configure — if the voice demo works, this works.

```shell
# Same AWS creds as the voice demo (Bedrock access):
export AWS_ACCESS_KEY_ID=...
export AWS_SECRET_ACCESS_KEY=...
export AWS_DEFAULT_REGION=us-east-1

python -m voice_agent.main      # then say "what do you see?"
```

An empty question gives a general description; a specific one (e.g. *"what's on
the table?"*) is answered specifically. Tuning lives in env vars read by
[voice_agent/config.py](voice_agent/config.py) — `SCENE_MODEL_ID`,
`SCENE_REGION`, `SCENE_MAX_TOKENS`, `SCENE_SYSTEM_PROMPT`, `SCENE_DEFAULT_PROMPT`.
If Bedrock is unreachable or no camera frame has arrived yet, the tool returns a
short error the agent relays instead of failing silently.

> Only the **ROS2 transport** (`python -m voice_agent.main`) exposes the camera
> over the bridge, so scene description works there; on the direct-WebRTC path
> (`main_sonic.py`) the tool returns a helpful error.

## "What did you see earlier?" — robot memory (temporal Q&A)

The robot remembers what it has looked at. Every time you ask *"what do you
see?"*, the agent snapshots a **timestamped (image, description)** into an
in-memory log. You can then ask about the **past**, and the `recall_scene` tool
finds the snapshot nearest the requested time, re-queries that archived image,
and answers from memory:

```
🐕 > what do you see?
🤖 A laptop and two coffee cups on the table.
   … (objects get moved around) …
🐕 > what was on the table 30 seconds ago?
🤖 About 30 seconds ago the table had a laptop and two coffee cups.
🐕 > and what about two minutes ago?
🤖 Two minutes ago it was empty.
```

The agent converts phrases like *"a minute ago"* → 60s, *"two minutes ago"* →
120s and picks the closest snapshot (`recall_scene`). Memory is **per session**
— it lives only while the voice agent runs, and holds up to `SCENE_MEMORY_MAX`
snapshots (default 120). Set `SCENE_MEMORY_DIR` to also persist each snapshot
JPEG to disk (e.g. for a monitor to display the recalled frame alongside the
answer). If nothing was observed near that time, the robot says so rather than
inventing a memory.

**Spot the difference.** Asking what *changed* (rather than what *was*) puts two
images in front of the model at once — the earlier snapshot **and** a live frame
— via the `compare_scenes` tool:

```
🐕 > what's different on the table now?
🤖 One of the coffee cups is gone — it's down to a laptop and a single cup.
🐕 > did anything move since a minute ago?
🤖 The laptop slid to the left and someone added a notebook.
```

This is the only path that actually compares two frames; `describe_scene` and
`recall_scene` each look at a single image.

## KVS Camera Streaming

Streams the robot's camera to **AWS Kinesis Video Streams** so you can watch it —
over HLS, from anywhere, with 24 hours of retention to scrub back through.

Two voice commands read back off the stream, both served by the `SceneDescriber`
Lambda (GetClip + ffmpeg → Bedrock, text returned for Nova Sonic to speak):

| Say | Lambda does |
| --- | --- |
| *"kvs describe scene"* | Newest frame → describe. Duplicates `describe_scene` on purpose — the stream route is the point when demoing. |
| *"kvs compare scene with 10 minutes ago"* | Frame from the archive **and** a live frame → what changed. |

The compare one is the one worth having. `compare_scenes` (local) can only compare
against a snapshot the agent took earlier in the session, so it cannot answer about a
moment nobody asked about; the KVS version reads the 24-hour archive, so *"what
changed since this morning?"* works even if the agent was not running then.

**Both only work from the local agent** (`make voice`): the hosted runtime's subnets
have no route out to Lambda. On failure the agent falls back to the live-camera tools.

> A larger pipeline used to sit here — cloud recall, a scene monitor on a 1-minute
> schedule, IoT topic rules, and an MQTT leg that fed a ROS2 `/tts` node. All
> removed: the monitor ran at reserved concurrency 0 for months, recall never
> produced a log line, and Nova Sonic's speech-to-speech had already made the `/tts`
> node redundant. See [`deployment/README.md`](deployment/README.md).

### Enable KVS streaming in the container

Set `KVS_ENABLED=true` in `docker/.env` and add your AWS credentials:

```ini
KVS_ENABLED=true
KVS_STREAM_NAME=go2-robot-01-camera
AWS_DEFAULT_REGION=us-east-1
AWS_ACCESS_KEY_ID=<your key>
AWS_SECRET_ACCESS_KEY=<your secret>
AWS_SESSION_TOKEN=                  # leave blank for IAM user keys
```

Then restart the container:

```bash
cd docker
docker compose -f docker-compose.mac.yml down
docker compose -f docker-compose.mac.yml up -d
```

The `kvs_producer_node` will start automatically and begin streaming camera
frames to KVS in the background.

Which topic gets streamed is up to you — `KVS_IMAGE_TOPIC=/annotated_image` sends
the detector's overlay feed (what the cloud stack uses), the default
`/camera/image_raw` sends the plain camera. `KVS_SEGMENT_SEC` trades latency for
upload efficiency; the cloud stack uses 2.

To watch it, see [Watching the KVS video stream](README.md#watching-the-kvs-video-stream).

### Deploy the CDK stack

The KVS stream and the `SceneDescriber` Lambda are defined in
[`deployment/`](deployment/) and deployed once per AWS account/region — see
[The KVS stack](AWS_DEPLOYMENT.md#the-kvs-stack).

## Run the container in AWS

Deploying and operating the cloud stacks — the CDK stacks and one-time setup,
the daily `make` targets, the auto-stop budget, the gotchas, and how to debug a
robot that won't connect — is its own guide:
**[AWS_DEPLOYMENT.md](AWS_DEPLOYMENT.md)**.

## 3D raw pointcloud dump

To save raw LIDAR data, `export` the following:

```shell
export MAP_SAVE=True
export MAP_NAME="3d_map"
```

Every 10 seconds, pointcloud data (in `.ply` format) will be saved to the root folder of the repo. **NOTE**: This is _not_ a Nav2 map but a raw data dump of LIDAR data useful for low-level debugging.

## Multi robot support

If you want to connect several robots for collaboration:

```shell
export ROBOT_IP="robot_ip_1, robot_ip_2, robot_ip_N"
```

## Switching between webrtc connection (Wi-Fi) to CycloneDDS (Ethernet)

```shell
export CONN_TYPE="webrtc"
```
or
```
export CONN_TYPE="cyclonedds"
```

## Visualization (Lichtblick)

<p align="center">
<img width="1200" height="630" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/f0920d6c-5b7a-4718-b781-8cfa03a88095" alt='foxglove_bridge WebSocket viewer (screenshot shows visually equivalent v1 UI)'>
</p>

We use [Lichtblick](https://github.com/lichtblick-suite/lichtblick) (Apache 2.0)
as the recommended viewer. Lichtblick is a community fork of Foxglove Studio
v1 maintained by BMW; it speaks the same `foxglove_bridge` WebSocket protocol
and ships the same panels (3D, Plot, Image, Teleop, Raw Messages). We chose
it over Foxglove Studio because Foxglove Studio's MPL 2.0 + commercial
licensing terms are not compatible with this project's distribution
requirements; Lichtblick is fully OSS and protocol-compatible with the same
`foxglove_bridge` ROS package (Apache 2.0) we already depend on.

See [Connect Lichtblick (OSS viewer)](#connect-lichtblick-oss-viewer) above
for connection steps.

## WebRTC Topic Interface

The SDK provides a WebRTC topic interface that allows sending various commands to the robot. This is particularly useful for non-movement actions such as turning on headlights, playing sounds, and other robot control functions.

To send commands via the WebRTC topic:

```bash
# Basic command structure
ros2 topic pub /webrtc_req go2_interfaces/msg/WebRtcReq "{api_id: <API_ID>, parameter: '<PARAMETER>', topic: '<TOPIC>', priority: <0|1>}" --once

# Example: Send a handshake command
ros2 topic pub /webrtc_req go2_interfaces/msg/WebRtcReq "{api_id: 1016, topic: 'rt/api/sport/request'}" --once
```

## WSL 2

If you are running ROS2 under WSL2 - you may need to configure Joystick\Gamepad to navigate the robot.

1. Step 1 - share device with WSL2

    Follow steps here https://learn.microsoft.com/en-us/windows/wsl/connect-usb to share your console device with WSL2

2. Step 2 - Enable WSL2 joystick drivers

    WSL2 does not come by default with the modules for joysticks. Build WSL2 Kernel with the joystick drivers. Follow the instructions here: https://github.com/dorssel/usbipd-win/wiki/WSL-support#building-your-own-wsl-2-kernel-with-additional-drivers  If you're comfortable with WSl2, skip the export steps and start at `Install prerequisites.`

    Before buiding, edit `.config` file and update the CONFIG_ values listed in this GitHub issue: https://github.com/microsoft/WSL/issues/7747#issuecomment-1328217406

2. Step 3 - Give permissions to /dev/input devices

    Once you've finished the guides under Step 3 - you should be able to see your joystick device under /dev/input

    ```bash
    ls /dev/input
    by-id  by-path  event0  js0
    ```

    By default /dev/input/event* will only have root permissions, so joy node won't have access to the joystick

    Create a file `/etc/udev/rules.d/99-userdev-input.rules` with the following content:
    `KERNEL=="event*", SUBSYSTEM=="input", RUN+="/usr/bin/setfacl -m u:YOURUSERNAME:rw $env{DEVNAME}"`

    Run as root: `udevadm control --reload-rules && udevadm trigger`

    https://askubuntu.com/a/609678

3. Step 3 - verify that joy node is able to see the device properly.

    Run `ros2 run joy joy_enumerate_devices`

    ```
    ID : GUID                             : GamePad : Mapped : Joystick Device Name
    -------------------------------------------------------------------------------
    0 : 030000005e040000120b000007050000 :    true :  false : Xbox Series X Controller
    ```

## Your feedback and support mean the world to us

If you're as enthusiastic about this project as we are, please consider giving
the [upstream project](https://github.com/abizovnuralem/go2_ros2_sdk) a :star:
star!

## Thanks

Special thanks to:
1. @tfoldi (Tamas) for his idea and talent to create a webrtc connection method between python and unitree GO2;
2. @budavariam for helping with lidar issues;
3. @legion1581 for a new webrtc method, that is working with 1.1.1 firmware update;
4. @alex.lin for his passion in ros1 ingration;
5. @alansrobotlab for his passion in robotics and helping me to debug new webrtc method;
6. @giangalv (Gianluca Galvagn) for helping me debug new issues with webrtc;
7. Many many other open source contributors! and TheRoboVerse community!

## License

Amazon's contributions to this repository are licensed under the **MIT-0** License
— see [LICENSE](LICENSE).

This repository is a fork of the RoboVerse community's
[go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) and redistributes
that project's source in-tree under its own terms, along with `coco_detector`
under MIT. MIT-0 waives attribution for Amazon's contributions only — it does not
waive the attribution those upstream licenses require. **If you redistribute this
repository you must retain their notices**, which are reproduced in full in
[THIRD-PARTY-LICENSES](THIRD-PARTY-LICENSES). That file also records an upstream
licensing inconsistency (the project declares BSD-2-Clause, BSD-3-Clause, and
Apache-2.0 in different places) and one vendored binary whose provenance could not
be established.
