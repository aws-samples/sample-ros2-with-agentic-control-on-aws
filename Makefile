# Go2 ROS2 SDK — task runner.
#
# Front door for the long commands documented in DEVELOPER_GUIDE.md and
# AWS_DEPLOYMENT.md. Run
# `make` (or `make help`) for the list.
#
# This file is also the single place the AWS profile/region are pinned, so
# `make <target>` behaves the same from an external terminal, a VS Code
# integrated terminal, and a VS Code task. All of those are set with
# `?=`/conditional assignment, so an ambient value from your shell still wins.

SHELL := /bin/bash
.DEFAULT_GOAL := help

# Non-interactive shells (VS Code tasks) don't source your shell rc file, so a
# directory your AWS `credential_process` helper lives in may be missing from
# PATH — every aws call then fails with a credential error. Point
# GO2_EXTRA_PATH at that directory to have it prepended here:
#   make deploy GO2_EXTRA_PATH=$HOME/bin
GO2_EXTRA_PATH ?=
ifneq ($(GO2_EXTRA_PATH),)
export PATH := $(GO2_EXTRA_PATH):$(PATH)
endif

# Region and profile are pinned with := rather than ?= *on purpose*, so an
# AWS_REGION/AWS_PROFILE already exported in your shell cannot silently
# redirect a stack lookup to the wrong region or account. (The AWS CLI honors
# AWS_REGION over AWS_DEFAULT_REGION, which is how that bug usually lands.)
# Override per-invocation instead: `make tunnel GO2_REGION=us-west-2`.
GO2_REGION  ?= us-east-1
export AWS_REGION := $(GO2_REGION)
export AWS_DEFAULT_REGION := $(GO2_REGION)

# Left empty so ambient credentials (env vars, instance role, SSO default) work
# out of the box. Set it to use a named profile:
#   make deploy GO2_PROFILE=my-profile
GO2_PROFILE ?=
ifneq ($(GO2_PROFILE),)
export AWS_PROFILE := $(GO2_PROFILE)
endif

# Any docker-compatible CLI works here: `make up ENGINE=<other-engine>`.
ENGINE        ?= docker
CONTAINER     ?= docker-unitree_ros-1
PYTHON        ?= python3.13
NETWORK_STACK ?= Go2NetworkStack
EC2_STACK     ?= Go2Ec2Stack
KVS_STACK     ?= Go2KVSStack
# One Bedrock Guardrail, imported by every other stack — so it deploys first.
GUARDRAIL_STACK ?= Go2GuardrailStack
# Holds the robot container image. Owned by no stack on purpose (see push-image);
# Go2Ec2Stack references it by this name and push-image creates it if missing, so
# nothing has to exist before the first push.
ECR_REPO      ?= go2-ros2-sdk
# Holds the whole robot link as JSON: {"email", "password", "serial"}. The robot
# container reads it at start, so none of it lives in a stack parameter.
# Must match Go2Ec2Stack's UnitreeSecretName default.
UNITREE_SECRET ?= unitree_go2_account_password

# Compose is run from docker/ so .env is picked up. From the repo root
# AES_128_KEY lands empty in the container and the driver exits immediately
# with "This robot speaks data2=3 — the per-device AES-128 key is required".
COMPOSE := cd docker && $(ENGINE) compose -f docker-compose.mac.yml --env-file .env

# ROS2 CLI inside the container needs both overlays sourced; each recipe line
# is its own shell, so this has to stay &&-chained onto the command.
EXEC      := $(ENGINE) exec $(CONTAINER) bash -lc
ROS_SETUP := source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash

# Unitree sport-mode command over /webrtc_req. $(call sport,<api_id>)
define sport
$(EXEC) "$(ROS_SETUP) && ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq '{api_id: $(1), parameter: \"\", topic: \"rt/api/sport/request\"}'"
endef

# Read ROBOT_IP out of docker/.env. Takes the first whitespace-delimited token
# so the inline comment is dropped -- a literal '#' can't appear here, make
# would treat it as the start of a comment even inside $(shell ...).
ROBOT_IP := $(shell awk -F= '/^ROBOT_IP=/{print $$2}' docker/.env 2>/dev/null | awk '{print $$1}')

# maps/, docker/ and scripts/ are real directories — without .PHONY make treats
# same-named targets as up-to-date files and silently does nothing.
.PHONY: help up down restart recreate logs logs-errors shell \
        stand balance stop sit standdown hello crouch drive battery \
        detections \
        venv test lock audit voice voice-text voice-cloud voice-ec2 \
        voice-agentcore voice-sim voice-sim-text sim-demo \
        find-robot check-robot whoami \
        tunnel open-ec2 close-ec2 start-ec2 stop-ec2 restart-ec2 status-ec2 \
        extend-ec2 autostop-ec2 \
        cdk-install check-secret deploy-guardrail deploy-kvs deploy-cloud deploy-network \
        deploy-ec2 push-image deploy-agentcore destroy-ec2 ec2-remote ec2-bridge \
        ap-probe ap-bridge ap-bridge-check

help:  ## Show this help
	@echo "Go2 ROS2 SDK — make targets (profile: $(AWS_PROFILE), region: $(AWS_REGION))"
	@awk 'BEGIN{FS=":.*?## "} \
	     /^## /{printf "\n\033[1m%s\033[0m\n", substr($$0,4)} \
	     /^[a-zA-Z0-9_-]+:.*?## /{printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo

## Local container (Mac / Docker)

up:  ## Build + start the container (first build ~30 min, then seconds)
	$(COMPOSE) up -d --build

down:  ## Stop and remove the container
	$(COMPOSE) down

restart:  ## Restart the driver — drops a stuck WebRTC session cleanly
	$(COMPOSE) restart

recreate:  ## Full down+up — REQUIRED after editing docker/.env (restart won't re-read it)
	$(COMPOSE) down
	$(COMPOSE) up -d

logs:  ## Follow the driver log; ready at "Robot 0 validated and ready"
	tail -f docker/logs/robot.log | grep -E "validated|Failed|Successfully"

logs-errors:  ## Last 30 log lines, errors only (use when Lichtblick shows no topics)
	$(COMPOSE) logs --tail=30 unitree_ros | grep -iE "error|failed|closed|validated"

shell:  ## Interactive shell in the container with ROS sourced
	$(ENGINE) exec -it $(CONTAINER) bash -lc "$(ROS_SETUP) && exec bash"

## Driving the robot (stand THEN balance, or /cmd_vel_out is ignored)

stand:  ## 1004 StandUp — joints lock, dog rises rigidly
	$(call sport,1004)

balance:  ## 1002 BalanceStand — REQUIRED before walking
	$(call sport,1002)

stop:  ## 1003 StopMove — halt current motion
	$(call sport,1003)

sit:  ## 1009 Sit
	$(call sport,1009)

standdown:  ## 1005 StandDown
	$(call sport,1005)

hello:  ## 1016 Hello — paw wave
	$(call sport,1016)

crouch:  ## 1013 BodyHeight -0.18 (lowest)
	$(EXEC) "$(ROS_SETUP) && ros2 topic pub --once /webrtc_req go2_interfaces/msg/WebRtcReq '{api_id: 1013, parameter: \"{\\\"data\\\":-0.18}\", topic: \"rt/api/sport/request\"}'"

drive:  ## Walk forward until Ctrl+C (X=0.3 Z=0.0 to override)
	$(EXEC) "$(ROS_SETUP) && ros2 topic pub /cmd_vel_out geometry_msgs/msg/Twist '{linear: {x: $(or $(X),0.3)}, angular: {z: $(or $(Z),0.0)}}'"

battery:  ## Print battery state of charge (%)
	$(EXEC) "$(ROS_SETUP) && ros2 topic echo --once --field bms_state.soc /lowstate"

## Object detection

# There is no `detector` target: robot.launch.py starts coco_detector_node itself
# (coco defaults to true), so `ros2 run`-ing another one gave two same-named nodes
# double-publishing /detected_objects and /annotated_image with two copies of
# Faster R-CNN competing for CPU. The backbone is loaded once in the node's
# constructor and can't be changed with `ros2 param set`, so switching it means
# recreating the container — which is just `make up` with the compose vars set:
#
#     COCO_MODEL=accurate COCO_THRESHOLD=0.4 make up
#
# (fast | balanced | accurate | best). Persist a choice in docker/.env instead.
detections:  ## Echo /detected_objects
	$(EXEC) "source /ros_entrypoint.sh && ros2 topic echo /detected_objects"

## Voice agent (host-side, needs AWS creds)

venv:  ## Create .venv and install every host-side dep (all voice targets use it)
	@# One venv for all four entry points. They were once split (.venv-voice for
	@# the agent, .venv-ac-client for the thin client) but the two dependency
	@# sets resolve together cleanly, and the split cost ~600 MB and a constant
	@# "which venv was that in?" tax. The container images are unaffected: they
	@# install requirements.txt / requirements-agentcore.txt, never this venv.
	@#
	@# Installs the COMPILED closure, not voice_agent/requirements.in — so a fresh
	@# venv gets the same transitive versions the last `make lock` resolved.
	$(PYTHON) -m venv .venv
	@# --require-hashes verifies every artifact against the sha256 in the compiled
	@# file. Regenerate with `make lock` after editing any .in file.
	.venv/bin/pip install --require-hashes -r voice_agent/requirements.txt

lock:  ## Recompile all four requirements.txt from their .in files (needs uv)
	@# The dependency files come in pairs: a hand-written .in (direct deps, pinned,
	@# commented) and a generated .txt (the full transitive closure). Everything
	@# that installs — both Dockerfiles and `make venv` — reads the .txt, and so
	@# does `make audit`. That is the whole point: the file that gets scanned is the
	@# file that gets installed.
	@#
	@# Each target compiles for the platform it DEPLOYS to, not this laptop, so the
	@# closure matches the image rather than the machine that generated it. Add
	@# --upgrade to move transitive versions; without it, existing pins are kept.
	@# --generate-hashes pins the ARTIFACT, not just the version. Version pinning
	@# gives reproducibility; hashes are what make a compromised index or a
	@# re-uploaded wheel fail the install instead of executing. Every install path
	@# passes --require-hashes to enforce them, which also refuses any dependency
	@# that is not pinned — so the two settings have to move together.
	uv pip compile requirements.in --generate-hashes \
	  --python-platform x86_64-manylinux_2_28 --python-version 3.10 \
	  -o requirements.txt
	@# Same image, other architecture. The two closures are NOT interchangeable:
	@# `torch==2.14.0` is a CUDA build on x86_64 (what the g4dn EC2 target wants) and
	@# a Grace/sbsa CUDA build on aarch64 that segfaults on import under Docker
	@# Desktop on Apple Silicon, so the local build needs the +cpu wheels. Full
	@# reasoning in requirements-arm64.in; docker/Dockerfile picks by `uname -m`.
	@#
	@# --index-strategy unsafe-best-match: torch comes from download.pytorch.org and
	@# everything else from PyPI, and the default strategy stops at the first index
	@# that has the name at all. It does not relax pins or hashes. --emit-index-url
	@# writes that extra index into the compiled file so `pip install -r` can reach
	@# the +cpu wheels without the Dockerfile repeating the URL.
	uv pip compile requirements-arm64.in --generate-hashes \
	  --emit-index-url --index-strategy unsafe-best-match \
	  --python-platform aarch64-manylinux_2_28 --python-version 3.10 \
	  -o requirements-arm64.txt
	uv pip compile voice_agent/requirements.in --generate-hashes \
	  --python-platform aarch64-apple-darwin --python-version 3.12 \
	  -o voice_agent/requirements.txt
	uv pip compile voice_agent/requirements-agentcore.in --generate-hashes \
	  --python-platform aarch64-manylinux_2_28 --python-version 3.13 \
	  -o voice_agent/requirements-agentcore.txt
	@echo
	@echo "[i] Recompiled. Commit the .in and .txt together, then run 'make audit'."

audit:  ## Check every pinned dependency for known CVEs (OSV + npm audit)
	@# "We pinned everything" is a reproducibility claim, not a security one, so
	@# something has to actually check. Needs no tooling installed — scripts/
	@# audit_deps.py talks to api.osv.dev over stdlib HTTP. Fails on any advisory
	@# that is not explicitly accepted in that script, with the reason.
	@#
	@# Reads the three compiled requirements.txt — the transitive closure, and the
	@# same files the images install. To fix a finding: bump the pin in the .in,
	@# `make lock`, re-run this.
	python3 scripts/audit_deps.py
	@echo
	@echo "[*] npm (CDK infrastructure)..."
	cd deployment && npm audit
	@echo
	@echo "[i] Not covered here: OS packages from the container base images, and"
	@echo "    the vendored libvoxel.wasm (a provenance question, not a CVE — see"
	@echo "    THIRD-PARTY-LICENSES). For those, generate an SBOM and scan it:"
	@echo "      syft . -o cyclonedx-json > sbom.json && grype sbom:sbom.json"

test:  ## Offline tests: safety controls, greeter, object-find, and the MuJoCo sim
	@# The safety suite first, and it is the fast one: the two-step gate in front of
	@# flips, `move`'s refusal to default to forward, and the robot-event trust
	@# boundary. These are security controls, so they are asserted rather than
	@# commented — see the docstring in tests/test_safety_controls.py.
	.venv/bin/python tests/test_safety_controls.py
	@# Pure-simulation suites: a fake robot that only stops on StopMove and a
	@# detector on an offset ROS clock. No robot, no AWS, no ROS2 — safe to run
	@# anywhere, takes ~2 min because the sweeps are time-driven.
	.venv/bin/python tests/test_greeter_logic.py
	.venv/bin/python tests/test_find_object_logic.py
	@# The sim suite drives real MuJoCo physics and asserts on measured hip
	@# heights and distances, because a mistuned pose or a base that ghosts
	@# through the furniture still returns success. ~1 min.
	.venv/bin/python tests/test_sim_dog.py

voice:  ## Voice + text agent against the local container (bridge on 8766)
	.venv/bin/python -m voice_agent.main

voice-text:  ## Same agent, text only (no mic)
	.venv/bin/python -m voice_agent.main --text-only

voice-sim:  ## Voice + text agent against a MuJoCo Go2 — no robot, no container
	@# The only voice target that needs no hardware, no container and no Unitree
	@# credentials: the dog is simulated (Strands Labs robots + MuJoCo, see
	@# voice_agent/sim_dog.py). AWS credentials are still needed for Nova Sonic.
	@#
	@# mjpython, not python: MuJoCo's viewer window must own the UI thread on
	@# macOS and plain python cannot give it one, so `python -m voice_agent.main_sim`
	@# runs fine but headless. mjpython ships with the mujoco wheel and is a
	@# no-op wrapper elsewhere. Extra flags go in ARGS, e.g.
	@# `make voice-sim ARGS="--scene empty"`.
	.venv/bin/mjpython -m voice_agent.main_sim $(ARGS)

voice-sim-text:  ## Same simulated dog, text only (no mic)
	.venv/bin/mjpython -m voice_agent.main_sim --text-only $(ARGS)

sim-demo:  ## Scripted sim run — stand, walk, sit, wave, dance. No agent, no AWS
	@# Proves the simulation itself works without involving Nova Sonic, a mic or
	@# credentials: the fastest way to tell a broken sim from a broken agent.
	@# SAVE=1 also writes a head-camera and room-camera PNG per step.
	.venv/bin/mjpython -m voice_agent.main_sim --demo $(if $(SAVE),--save-frames,) $(ARGS)

voice-cloud:  ## Voice agent against the cloud robot through the SSM tunnel (port 9876)
	@# Needs 'make tunnel' running in another terminal. This is the laptop-side
	@# agent; 'make voice-agentcore' talks to the hosted one, which reaches the
	@# robot's private IP from inside the VPC and needs no tunnel.
	ROS_BRIDGE_HOST=localhost ROS_BRIDGE_PORT=$(or $(GO2_LOCAL_PORT),9876) .venv/bin/python -m voice_agent.main

voice-ec2:  ## Voice agent straight to the EC2 robot on :8765 — needs 'make open-ec2' first
	@# Same agent as voice-cloud, but pointed at the instance's public address
	@# instead of a local tunnel port, so camera grabs and detections don't relay
	@# through the SSM service. Requires the temporary /32 rule that 'make open-ec2'
	@# adds; ec2-ctl.sh host refuses (with the fix) if it isn't there.
	@#
	@# The address is resolved inside the recipe, not with $$(shell ...) at parse
	@# time — otherwise every `make <anything>` would call EC2 and it would break
	@# when the stack isn't deployed.
	@#
	@# Extra agent flags go in ARGS: `make voice-ec2 ARGS=--text-only`.
	@set -euo pipefail; \
	HOST=$$(./scripts/ec2-ctl.sh host); \
	echo "[*] Voice agent -> ws://$$HOST:8765 (direct, no tunnel)"; \
	ROS_BRIDGE_HOST=$$HOST ROS_BRIDGE_PORT=8765 \
	  .venv/bin/python -m voice_agent.main $(ARGS)

voice-agentcore:  ## Talk to the hosted AgentCore runtime (no tunnel needed)
	.venv/bin/python -m voice_agent.agentcore_client

## Finding the robot

find-robot:  ## Find the dog's current IP by MAC (its WiFi IP drifts; the app lags)
	arp -a | grep -i "78:22:88" || echo "Not in ARP cache — ping the last-known IP first, then retry."

check-robot:  ## Check this Mac can reach the robot's signaling port
	@test -n "$(ROBOT_IP)" || { echo "ROBOT_IP not found in docker/.env"; exit 1; }
	nc -z -v -G 5 $(ROBOT_IP) 9991

whoami:  ## Show which AWS identity the make targets will use
	aws sts get-caller-identity

## Cloud — daily use

# The robot runs on one EC2 instance in the shared VPC. Stopped, the whole thing
# costs ~$8/mo of EBS (which is what keeps starts at ~40s instead of a cold 14 GB
# pull) and the VPC itself is free — there is no NAT gateway any more.
# Running, it is a g4dn.xlarge at ~$0.526/hr — the T4 is what makes the accurate
# detector backbone usable, so stop it when you're done. If you forget, it stops
# itself 4 hours after each start (see extend-ec2 / autostop-ec2 below).

start-ec2:  ## Boot the robot instance (ready in ~40s — image layers are cached)
	./scripts/ec2-ctl.sh start

stop-ec2:  ## Stop the instance — revokes direct access, compute billing ends
	@# Ctrl-C any open tunnel FIRST: stopping the instance kills the session's
	@# remote end but leaves session-manager-plugin holding the local port, and
	@# the next client then connects to a dead socket and just hangs.
	./scripts/ec2-ctl.sh stop

restart-ec2:  ## Roll a freshly pushed image in place (pull + restart, no replacement)
	./scripts/ec2-ctl.sh restart

status-ec2:  ## Instance state, auto-stop countdown, container state, link mode, unit log
	./scripts/ec2-ctl.sh status

# The instance stops ITSELF 4 hours after each start — a watchdog Lambda in
# Go2Ec2Stack polls every 5 minutes — because a forgotten `make stop-ec2` is the
# only thing here that can cost real money (~$0.526/hr), and an unattended robot
# dialing Unitree's cloud is also how the source IP gets WAF-blocked.
#
# Both knobs are instance tags, so neither needs a deploy. `make status-ec2` prints
# how long is left.
extend-ec2:  ## +60 min on this session's auto-stop (MINUTES=90 for another amount)
	./scripts/ec2-ctl.sh extend $(MINUTES)

autostop-ec2:  ## Set the uptime budget: MINUTES=480, off to disarm, default to reset
	@# Persists across stop/start, unlike extend-ec2. An empty MINUTES falls through
	@# to the script, which prints the usage and the current setting and exits 1 —
	@# cheaper than a guard here, which would need a full status round-trip to say
	@# anything useful.
	./scripts/ec2-ctl.sh autostop $(MINUTES)

# Who owns the robot link. Switching is a container restart (~30s), NOT an instance
# replacement — the mode lives in /etc/go2.mode and go2-run.sh reads it at start.
#
# In bridge mode the instance does not fetch the Unitree secret at all, so it holds
# no credentials and cannot contact global-robot-api.unitree.com. That is what keeps
# it from adding to the EdgeOne WAF blocks while the dog is on its own AP.
ec2-remote:  ## EC2 owns the robot link via Unitree's TURN server (needs the dog online)
	./scripts/ec2-ctl.sh mode remote

ec2-bridge:  ## Laptop owns the link (AP mode); EC2 consumes only and never calls Unitree
	@# Requires an image that contains the CONN_TYPE=bridge support in
	@# go2_robot_sdk/launch/robot.launch.py. `make deploy-ec2` does NOT rebuild the
	@# image, so on an older one the unit fails with a ROS parameter tuple error and
	@# "Robot IPs: []". `make push-image && make restart-ec2` fixes it. The mode verb
	@# prints the unit log and says so if that happens.
	./scripts/ec2-ctl.sh mode bridge

tunnel:  ## SSM port-forward to the bridge on ws://localhost:9876 — leave open
	./scripts/tunnel.sh

open-ec2:  ## ESCAPE HATCH: expose the UNAUTHENTICATED bridge to your /32 (prefer 'make tunnel')
	@# NOT a normal step — 'make tunnel' is. The SSM tunnel relays through the SSM
	@# service, so camera/pointcloud lag; this adds a /32 security-group rule for
	@# your current public IP instead and Lichtblick connects straight to the
	@# instance. What that publishes is an UNAUTHENTICATED ROS 2 control plane,
	@# gated only by source IP — which is shared on office NAT and CGNAT. It
	@# refuses to run without GO2_ACK_UNAUTHENTICATED_BRIDGE=yes. Revoked by
	@# stop-ec2, start-ec2 and close-ec2. See the header of scripts/ec2-ctl.sh.
	./scripts/ec2-ctl.sh open

close-ec2:  ## Revoke the direct-access rule (leaves the instance running)
	./scripts/ec2-ctl.sh close

## Cloud — deploys (operators)

cdk-install:  ## Install CDK deps from the lockfile, exactly (pnpm)
	@# --frozen-lockfile rather than a bare install: package.json pins every version
	@# exactly, and frozen FAILS on a lockfile that disagrees instead of quietly
	@# resolving past it. pnpm, not npm — pnpm-lock.yaml is the only lockfile here
	@# and npm would write a competing package-lock.json (a preinstall guard in
	@# package.json stops it). corepack picks the pinned pnpm from packageManager.
	cd deployment && pnpm install --frozen-lockfile

deploy-guardrail:  ## Deploy Go2GuardrailStack (the Bedrock Guardrail) — do this first
	@# Every other stack imports this one's guardrail id, and they all deploy with
	@# --exclusively (which does NOT pull dependencies in), so this has to exist
	@# before any of them. It is one resource and takes seconds.
	@#
	@# Attach the same guardrail to a LOCAL `make voice` run by exporting
	@# GUARDRAIL_ID from this stack's GuardrailId output — see voice_agent/README.md
	@# for the describe-stacks one-liner.
	cd deployment && npx cdk deploy $(GUARDRAIL_STACK) --exclusively

deploy-kvs: deploy-guardrail  ## Deploy the KVS camera stream stack
	cd deployment && npx cdk deploy $(KVS_STACK) --exclusively

check-secret:  ## Verify the Unitree secret has email + password + serial
	@# deploy-ec2 depends on this. It replaces the UNITREE_EMAIL/SERIAL argument
	@# guards that used to live there: with the values in the secret instead of on
	@# the command line, a missing key would otherwise surface as a systemd unit
	@# that exits — a journalctl dig instead of a one-line message. Prints key
	@# NAMES only, never values.
	@set -euo pipefail; \
	if ! json=$$(aws secretsmanager get-secret-value --secret-id $(UNITREE_SECRET) \
	     --query SecretString --output text 2>/dev/null); then \
	  echo "[!] Secret '$(UNITREE_SECRET)' not found in $(AWS_REGION). Create it:"; \
	  echo "    aws secretsmanager create-secret --name $(UNITREE_SECRET) \\"; \
	  echo "      --secret-string '{\"email\":\"you@example.com\",\"password\":\"...\",\"serial\":\"B42D...\"}'"; \
	  exit 1; \
	fi; \
	echo "$$json" | python3 -c 'import json,sys; d=json.load(sys.stdin); m=[k for k in ("email","password","serial") if not d.get(k)]; sys.exit("[!] Secret $(UNITREE_SECRET) is missing key(s): "+", ".join(m)) if m else print("[*] Secret $(UNITREE_SECRET) OK — keys: "+", ".join(sorted(d)))'

deploy-cloud: deploy-guardrail deploy-network deploy-ec2 deploy-agentcore  ## Deploy the whole cloud path, in order
	@echo "[*] Guardrail, network, robot, and hosted agent deployed. 'make status-ec2' to check the robot."

deploy-network:  ## Deploy Go2NetworkStack (the shared VPC) — do this first
	@# Rarely changes after the first deploy. Everything else imports its subnets,
	@# so CloudFormation will refuse to delete or renumber them while in use — which
	@# is the intended protection, not an obstacle.
	cd deployment && npx cdk deploy $(NETWORK_STACK) --exclusively

deploy-ec2: check-secret  ## Deploy Go2Ec2Stack (no args — robot link comes from the secret)
	@# Email, password, and serial all live in the Secrets Manager secret
	@# ($(UNITREE_SECRET)), read at container start. Nothing about the robot link
	@# is a stack parameter, so there is nothing to pass and nothing to keep in
	@# sync. To point at a different robot, edit the secret's "serial" key.
	@#
	@# --exclusively so this never reaches sideways into the VPC or the agent.
	@# Go2NetworkStack must already exist (make deploy-network) — its subnet IDs
	@# are imported here, and an unresolvable import fails the deploy outright.
	cd deployment && npx cdk deploy $(EC2_STACK) --exclusively
	@echo "[!] The instance boots the robot container on deploy. 'make status-ec2' to check,"
	@echo "    'make stop-ec2' when you're done, 'make start-ec2' to bring it back."

push-image:  ## Build + push the amd64 robot container image to ECR
	@# The repo URI is derived from account+region rather than read from a stack
	@# output, because the repository is deliberately owned by no stack: it was
	@# declared by the old Fargate stack at RETAIN, so destroying that left the
	@# real repository and its layers in place. Created on demand here, so a fresh
	@# account can push before any stack exists.
	@set -euo pipefail; \
	ACCT=$$(aws sts get-caller-identity --query Account --output text); \
	ECR_URI="$$ACCT.dkr.ecr.$(AWS_REGION).amazonaws.com/$(ECR_REPO)"; \
	if ! aws ecr describe-repositories --repository-names $(ECR_REPO) >/dev/null 2>&1; then \
	  echo "[*] Creating ECR repository $(ECR_REPO)";	 \
	  aws ecr create-repository --repository-name $(ECR_REPO) \
	    --image-scanning-configuration scanOnPush=true >/dev/null; \
	fi; \
	echo "[*] Pushing to $$ECR_URI"; \
	aws ecr get-login-password | $(ENGINE) login --username AWS --password-stdin "$${ECR_URI%/*}"; \
	$(ENGINE) build --platform linux/amd64 -f docker/Dockerfile -t "$$ECR_URI:latest" .; \
	$(ENGINE) push "$$ECR_URI:latest"; \
	echo "[*] Pushed. Roll it onto the running robot: make restart-ec2"

deploy-agentcore:  ## Deploy the voice agent to Bedrock AgentCore Runtime (ARM64)
	@# Region, approval, and the voice/device/model-region knobs all come from
	@# cdk.json, so this needs no env-var preamble or --parameters.
	@#
	@# --exclusively matters here: this stack declares a dependency on Go2Ec2Stack
	@# (ordering only — it imports nothing from it), and without the flag CDK would
	@# deploy that too. A template change there replaces the instance, so deploying
	@# the agent could otherwise kill a robot mid-session.
	cd deployment && npm run deploy:agent
	@echo "[*] The runtime ARN and log group are in the stack outputs above;"
	@echo "    the client reads the ARN itself: make voice-agentcore"

destroy-ec2:  ## Tear the EC2 stack down — deletes the instance AND its cached image layers
	cd deployment && npx cdk destroy $(EC2_STACK)

## Inverted AP mode — laptop owns the dog link, EC2 consumes (no local container)

# The EC2 container cannot hold the robot link in AP mode: ICE offers only host
# candidates and the dog is its own gateway with no uplink, so EC2's 10.x address is
# unreachable. Relaying it needs a TURN server the laptop must ACCEPT inbound on,
# and corporate pf (Amazon + CrowdStrike anchors) drops that. So invert it: every
# connection is laptop-initiated, which pf permits.
#
#   dog --WebRTC LocalAP--> laptop bridge --SSM tunnel--> EC2 foxglove_bridge -> ROS graph
#                                  <--- /cmd_vel_out, /webrtc_req ---
#
# Deploy the container with CONN_TYPE=bridge so it does NOT fight for the dog's
# single WebRTC slot (go2_driver_node then skips connect_robots and the reconnect
# watchdog, but still keeps the command topics present in the graph).
#
#   terminal 1:  make tunnel
#   terminal 2:  make ap-bridge
#   then:        make voice-cloud   (or AgentCore, or Lichtblick on ws://localhost:9876)

ap-bridge:  ## Laptop holds the dog link and feeds EC2's ROS graph — needs `make tunnel`
	@# Defaults to the tunnel on localhost:9876. Set DIRECT=1 to skip the tunnel and
	@# publish straight to the instance's public :8765 instead — the same trade
	@# voice-ec2 makes, and it matters more here: the point cloud and camera are the
	@# bulk of this traffic, and the SSM relay is the slow link.
	@# Requires the temporary /32 rule from `make open-ec2`.
	@set -euo pipefail; 	if [ -n "$(DIRECT)" ]; then 	  HOST=$$(./scripts/ec2-ctl.sh host); 	  echo "[*] AP bridge -> ws://$$HOST:8765 (direct, no tunnel)"; 	  .venv/bin/python scripts/go2_ap_bridge.py 	    --bridge "ws://$$HOST:8765" $(ARGS); 	else 	  .venv/bin/python scripts/go2_ap_bridge.py $(ARGS); 	fi

ap-bridge-check:  ## Bring up both links, report, exit (no streaming)
	.venv/bin/python scripts/go2_ap_bridge.py --dry-run

ap-probe:  ## Diagnostic: can the dog answer an off-subnet peer? (30s, free)
	./scripts/ap_offsubnet_probe.sh
