// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: MIT-0
import * as cdk from "aws-cdk-lib";
import * as ec2 from "aws-cdk-lib/aws-ec2";
import * as ecr from "aws-cdk-lib/aws-ecr";
import * as events from "aws-cdk-lib/aws-events";
import * as targets from "aws-cdk-lib/aws-events-targets";
import * as iam from "aws-cdk-lib/aws-iam";
import * as lambda from "aws-cdk-lib/aws-lambda";
import * as logs from "aws-cdk-lib/aws-logs";
import * as secretsmanager from "aws-cdk-lib/aws-secretsmanager";
import { NagSuppressions } from "cdk-nag";
import { Construct } from "constructs";
import { applySolutionMetadata, solutionUserAgent } from "./solution";

export interface Go2Ec2StackProps extends cdk.StackProps {
  /**
   * Robot identifier — names the KVS stream (`<deviceId>-camera`) the container
   * writes to. Resolved once in bin/app.ts and shared with Go2KVSStack (which
   * creates the stream) and Go2AgentCoreStack (whose kvs_* tools read it).
   */
  readonly deviceId: string;
  /** The shared VPC from Go2NetworkStack. */
  readonly vpc: ec2.IVpc;
  /** Public subnet the instance goes in — must contain robotPrivateIp. */
  readonly robotSubnet: ec2.ISubnet;
  /**
   * Fixed private address for the instance. Pinned (rather than letting EC2
   * assign one) so Go2AgentCoreStack can address the bridge with a literal
   * instead of a CloudFormation import — see the note on ROBOT_PRIVATE_IP in
   * go2-network-stack.ts.
   */
  readonly robotPrivateIp: string;
}

/**
 * Runs the Go2 ROS2 SDK container on a single EC2 instance in the shared VPC.
 *
 * This is the only robot compute path. It replaced an ECS Fargate stack that
 * needed a NAT gateway, an internal NLB, and an SSM jumpbox — roughly $57/month
 * of always-on scaffolding — none of which bought anything except working around
 * two properties of a Fargate task: it has no host, and no stable address. An
 * instance is both, so all three went away:
 *
 *   - no NLB: the bridge is at a fixed private IP, so nothing needs a stable
 *     address put in front of it
 *   - no jumpbox: SSM cannot port-forward to a Fargate task, only to an instance,
 *     and this instance is one
 *   - no NAT on this path: a public subnet + internet gateway is free egress
 *
 * What is left is one box whose security group allows exactly one thing inbound:
 * the foxglove bridge on 8765, from inside the VPC only. That is how the hosted
 * agent reaches it. From a laptop there is no inbound path at all — you tunnel in
 * over SSM, so access is gated by IAM (who can open a session), not by a port.
 * The bridge itself is unauthenticated, by design and on purpose.
 *
 * ## Two roles, one instance
 *
 *   1. The robot: publishes the ROS graph, streams the annotated feed to KVS, and
 *      serves foxglove_bridge on 8765 (--network host). WHERE the robot data comes
 *      from is switchable at runtime via /etc/go2.mode (`make ec2-remote` /
 *      `make ec2-bridge`), because it is read by go2-run.sh at every unit start:
 *        remote  this instance dials the dog through Unitree's TURN server
 *        bridge  a laptop on the dog's own AP owns the WebRTC link and feeds this
 *                graph through foxglove_bridge's clientPublish. The instance then
 *                skips the Unitree secret fetch entirely, so it holds no
 *                credentials and cannot reach Unitree's cloud at all — which is
 *                what keeps it out of the EdgeOne WAF blocks. See
 *                scripts/go2_ap_bridge.py.
 *      Switching costs a container restart, not an instance replacement.
 *   2. The SSM tunnel target: `scripts/tunnel.sh` forwards localhost:9876 straight
 *      to its 8765. One hop, no intermediate host.
 *
 * ## Costs
 *
 * ~$8/month stopped (a 100 GB EBS volume, sized by the GPU AMI's 75 GiB root and
 * the 14 GB image; it is what keeps starts at ~40s instead of a cold pull) and
 * ~$0.526/hr running on g4dn.xlarge. The hourly rate looks steep next to a CPU
 * instance but the instance spends most of its life stopped, so what it really
 * costs is ~44 cents more per hour of actual driving.
 *
 * The VPC's NAT (~$33/month) is not this stack's — it exists for the AgentCore
 * runtime, which cannot live in a public subnet. Drop the hosted agent, drop the
 * NAT with it, and the standing cost of the whole system is that one volume.
 *
 * ## Things to know
 *
 *   - It is a GPU instance, and the detector is why. See the InstanceType comment
 *     below; the short version is that the T4 is what makes the 46.7 mAP backbone
 *     usable at all. Fargate could never do this — it is CPU-only, full stop.
 *   - Public IP changes on every start. This is a feature: Unitree's cloud WAF
 *     blocks by source IP after a reconnect storm, and a stop/start is the only
 *     reliable way to clear it. If something ever needs to allowlist the robot,
 *     add an Elastic IP and accept that a block becomes sticky.
 *   - The private IP does NOT change across stop/start, which is what makes it
 *     safe for AgentCore to hardcode. It changes only on instance replacement.
 *   - One robot at a time. There is no `desiredCount: N` equivalent; several
 *     concurrent robots would mean going back to a scheduler.
 *   - Going back to CPU means four coordinated changes, not one: instanceType,
 *     the AMI, and dropping `--gpus all` + `coco_device:=cuda` from the run
 *     script. The GPU preflight in go2-run.sh will stop you halfway if you miss
 *     one. All four are constants in this file precisely so they move together.
 *
 * ## The 4-hour auto-stop
 *
 * At ~$0.526/hr, the expensive failure mode of this stack is not a bug — it is a
 * forgotten `make stop-ec2`. An instance left running overnight costs more than a
 * month of everything else here put together, and it is also a robot dialing
 * Unitree's cloud unattended, which is how the source IP gets WAF-blocked.
 *
 * So a watchdog Lambda polls every 5 minutes and stops the instance once it has
 * been running longer than AUTO_STOP_MINUTES (4h). It is deliberately OUT OF BAND
 * rather than a systemd timer on the box:
 *
 *   - it is a pure stack addition, so adding it did not touch user data and did
 *     not replace the instance (no cold 14 GB pull)
 *   - it still fires when the box is wedged — which is the case that actually
 *     costs money, since a hung instance cannot poweroff itself
 *
 * Nothing about it needs a deploy to adjust; all three knobs are instance TAGS,
 * set by `scripts/ec2-ctl.sh` (see `make extend-ec2` / `make autostop-ec2`):
 *
 *   go2:autostop-minutes  per-instance budget, overrides AUTO_STOP_MINUTES
 *   go2:autostop-until    absolute ISO-8601 UTC reprieve — extend a live session
 *   go2:autostop=off      disarm entirely (it will then run until you stop it)
 *
 * The clock is the instance's LaunchTime, which for an EBS-backed instance is the
 * last START, not the original launch — so every `make start-ec2` gets a fresh 4
 * hours with no state to reset anywhere.
 *
 * ## Operating it
 *
 *   make deploy-ec2        # once
 *   make push-image        # once per code change
 *   make start-ec2         # boots, ready ~40s; auto-stops 4h later
 *   make tunnel            # ws://localhost:9876 — leave open
 *   make restart-ec2       # roll a new image in place
 *   make ec2-bridge        # hand the robot link to a laptop on the dog's AP
 *   make ec2-remote        # take it back
 *   make extend-ec2        # +60 min when the auto-stop is about to bite
 *   make stop-ec2          # billing stops
 */
export class Go2Ec2Stack extends cdk.Stack {
  constructor(scope: Construct, id: string, props: Go2Ec2StackProps) {
    super(scope, id, props);

    applySolutionMetadata(
      this,
      "Unitree Go2 AWS robotics demo - the robot host: one GPU instance running " +
        "the ROS 2 stack, plus the auto-stop watchdog that bounds its cost"
    );

    const deviceId = props.deviceId;

    // --- Parameters -------------------------------------------------------
    // No credential parameters at all. Email, password, and serial all come out
    // of the one secret at container start, so nothing about the robot link sits
    // in plaintext in the template — CloudFormation parameters are readable by
    // anyone who can call describe-stacks. deviceId stays in shared context
    // because Go2KVSStack needs it at synth time to name the stream. Everything
    // below is deploy-shape config, not robot identity.
    const unitreeSecretName = new cdk.CfnParameter(this, "UnitreeSecretName", {
      type: "String",
      default: "unitree_go2_account_password",
      description:
        'Secrets Manager secret holding the Unitree account. JSON: {"email": "...", "password": "...", "serial": "..."}',
    });

    // Referenced by name, never created — and deliberately unmanaged. The
    // repository was originally declared by the (now deleted) Fargate stack at
    // RemovalPolicy.RETAIN, so destroying that stack left the real repository and
    // its 14 GB of layers in place. Adopting it into a stack now would need a
    // CloudFormation resource import; re-declaring it would just fail with
    // "repository already exists". Referencing it costs nothing and keeps the
    // image independent of every stack's lifecycle, which is arguably where it
    // belongs. `make push-image` creates it if it is ever missing.
    const ecrRepoName = new cdk.CfnParameter(this, "EcrRepoName", {
      type: "String",
      default: "go2-ros2-sdk",
      description: "ECR repository holding the go2-ros2-sdk image",
    });

    const imageTag = new cdk.CfnParameter(this, "ImageTag", {
      type: "String",
      default: "latest",
      description: "ECR image tag to run",
    });

    // g4dn.xlarge — 4 vCPU + one NVIDIA T4. The detector is the bottleneck in this
    // whole system and it is the reason for the GPU:
    //
    //   coco_detector_node's measured per-frame times on ONE CPU thread (see the
    //   MODELS table in coco_detector_node.py) are ~380 ms for the 'balanced'
    //   backbone and ~3900 ms for 'best'. The previous c6i.large was 2 vCPU on a
    //   SINGLE physical core, so it ran 'balanced' at roughly 3 fps and could not
    //   run 'best' at all. A T4 collapses those numbers and makes the 46.7 mAP
    //   backbone usable, which is the entire point.
    //
    // Two constraints this still has to respect:
    //   - x86_64. The image is built `--platform linux/amd64`, so Graviton (g5g)
    //     would not run it at all, not merely run it slowly.
    //   - Non-burstable. A Faster R-CNN on every frame is sustained load, which is
    //     exactly what drains a t-family instance's credits before throttling it
    //     to a 20% baseline mid-session.
    //
    // It is ~6x the hourly cost of c6i.large ($0.526 vs $0.085), which matters far
    // less than it looks: the instance only bills while running, and it spends most
    // of its life stopped. An hour of driving costs about 44 cents more.
    //
    // A CONSTANT, not a CfnParameter, and that distinction cost a debugging cycle.
    // CDK sends UsePreviousValue for parameters a deploy does not supply, so
    // changing a parameter's *default* does nothing to an already-deployed stack:
    // this went out as a GPU AMI on a c6i.large, with no GPU for the driver to
    // talk to. A constant is part of the template body, so the deployed value
    // always matches what is in git.
    //
    // It also should not be tunable per-deploy, because it cannot vary alone. The
    // instance type, the AMI below, `--gpus all`, and `coco_device:=cuda` in the
    // run script are one decision in four places. Overriding just this one gets
    // you an instance that fails the GPU preflight — which is at least loud, but
    // there is no reason to leave the trap lying there.
    const instanceType = "g4dn.xlarge";

    // The GPU AMI below ships a 75 GiB root snapshot (NVIDIA driver + CUDA), and
    // EBS cannot shrink below a snapshot — 50, which was fine on plain AL2023,
    // now fails the deploy outright. 100 leaves room for the 14 GB image plus
    // overlay scratch, which is what keeps starts at ~40s instead of a cold pull.
    // At gp3 that is ~$8/month, and it is the only cost while stopped.
    const volumeSizeGb = 100;

    // How long the instance may stay running before the watchdog below stops it.
    // Four hours is longer than any session anyone has actually needed and short
    // enough that a forgotten instance costs ~$2 instead of ~$13 a day.
    //
    // A constant rather than a CfnParameter for the same reason instanceType is
    // one: CDK sends UsePreviousValue for parameters a deploy does not supply, so
    // editing a parameter's default here would change nothing about the deployed
    // stack and the file would quietly lie. This is the DEPLOY-TIME default only —
    // per-session and per-instance changes go through the tags, which need no
    // deploy at all.
    const autoStopMinutes = 240;

    // Deep Learning Base OSS NVIDIA Driver GPU AMI (Amazon Linux 2023): NVIDIA
    // driver, Docker, and the NVIDIA Container Toolkit preinstalled, with no
    // framework — we bring our own torch in the image, so the much larger
    // PyTorch DLAMIs would be wasted bytes.
    //
    // Installing the driver into plain AL2023 from user data was the alternative
    // and is the fragile path: it is a kernel-module build that can break on a
    // kernel bump and adds minutes to first boot.
    //
    // Resolved from the SSM public parameter at DEPLOY time (a CloudFormation
    // dynamic reference), not at synth, so nothing caches into cdk.context.json.
    // The tradeoff: `latest` moves when AWS publishes a new DLAMI, and a deploy
    // that picks up a new AMI id REPLACES the instance and pays one cold 14 GB
    // pull. `make restart-ec2` — the routine path — never does. Pin the id here
    // if you want that to be impossible. (`latestAmazonLinux2023()` had exactly
    // the same property, so this is not new behaviour.)
    const gpuAmiParameter =
      "/aws/service/deeplearning/ami/x86_64/base-oss-nvidia-driver-gpu-amazon-linux-2023/latest/ami-id";

    // --- Secret reference (password fetched at container start) -----------
    const unitreeSecret = secretsmanager.Secret.fromSecretNameV2(
      this,
      "UnitreeSecret",
      unitreeSecretName.valueAsString
    );

    const repo = ecr.Repository.fromRepositoryName(
      this,
      "Go2Repo",
      ecrRepoName.valueAsString
    );

    // --- Networking: shared VPC, public subnet, one inbound rule ----------
    // The VPC comes from Go2NetworkStack rather than being built here. Egress is
    // via the internet gateway (free) to the TURN server, ECR, Bedrock, KVS, and
    // SSM — the VPC's NAT is for the AgentCore runtime and this instance never
    // touches it.
    const vpc = props.vpc;

    const sg = new ec2.SecurityGroup(this, "RobotSg", {
      vpc,
      description: "Go2 robot instance - foxglove bridge from in-VPC, egress all",
      allowAllOutbound: true,
    });

    // Exactly one inbound rule, and only from inside the VPC: this is how the
    // AgentCore runtime's ENIs reach the bridge. Nothing on the internet can,
    // despite the public IP — a laptop still comes in over SSM, which needs no
    // open port at all. Scoped to the VPC CIDR rather than the runtime's security
    // group on purpose: an SG-to-SG rule would be a cross-stack reference back
    // into Go2AgentCoreStack, and the point of the fixed private IP is that these
    // two stacks agree on addresses, not on each other's resources.
    sg.addIngressRule(
      ec2.Peer.ipv4(vpc.vpcCidrBlock),
      ec2.Port.tcp(8765),
      "Foxglove bridge from inside the VPC (AgentCore runtime)"
    );

    // --- Direct browser/Lichtblick access is NOT declared here -------------
    // SSM port-forwarding relays every byte through the SSM service, which adds
    // latency and caps throughput; the camera and pointcloud topics feel it. The
    // faster option is a direct TCP connection to 8765 on the public IP, which
    // needs an ingress rule for the operator's address.
    //
    // That rule is deliberately NOT in this template. It is added and removed at
    // the edges of a session by `scripts/ec2-ctl.sh open|close` — always as a
    // single-host /32, always tagged with a marker in its description, and revoked
    // automatically by `stop`. Reasons for keeping it out of CloudFormation:
    //
    //   - It is genuinely ephemeral. A home/office IP moves, and a `cdk deploy`
    //     per session (1-2 min) to chase it is worse than an instant CLI call.
    //   - Exposure should be bounded by the session, not by however long someone
    //     forgets to redeploy. `stop` cannot leave a rule behind.
    //   - A deploy reverting an out-of-band rule is the desired failure mode here,
    //     not a problem: worst case a session's rule vanishes and you re-run
    //     `make open`.
    //
    // What this DOES mean, and it is the whole risk: foxglove_bridge has no
    // authentication, so while that rule exists anything at that address can
    // publish /cmd_vel_out and /webrtc_req — i.e. drive the robot. A /32 is only
    // as narrow as the address is: behind a corporate NAT it covers every host
    // sharing that egress. SSM (make tunnel) has no such exposure and stays the
    // default.

    // --- Logs --------------------------------------------------------------
    // Docker's awslogs driver writes here, so `aws logs tail /go2/ec2 --follow`
    // gets you the ROS graph's stdout. Note the consequence: container output does
    // NOT stay on the box, so boot-time failures (ECR login, secret fetch, docker
    // itself) are NOT here — those land in `journalctl -u go2`, which is what
    // `make status-ec2` reads.
    const logGroup = new logs.LogGroup(this, "Go2LogGroup", {
      logGroupName: "/go2/ec2",
      retention: logs.RetentionDays.ONE_WEEK,
      removalPolicy: cdk.RemovalPolicy.DESTROY,
    });

    // --- Instance role ----------------------------------------------------
    const role = new iam.Role(this, "RobotRole", {
      assumedBy: new iam.ServicePrincipal("ec2.amazonaws.com"),
      description: "Go2 robot instance - SSM, ECR pull, secret read, KVS, Bedrock",
      managedPolicies: [
        // SSM Session Manager: the tunnel target and the only way in.
        iam.ManagedPolicy.fromAwsManagedPolicyName("AmazonSSMManagedInstanceCore"),
      ],
    });

    repo.grantPull(role);
    unitreeSecret.grantRead(role);

    // KVS streaming for kvs_producer_node, scoped to this robot's own stream.
    // The trailing wildcard is the stream's CREATION TIMESTAMP, which is the last
    // ARN segment for a KVS stream and is assigned by the service — so it cannot
    // be written out here. Go2KVSStack owns the stream, hence the name is rebuilt
    // from deviceId rather than imported: importing its ARN would couple the two
    // stacks' lifecycles for no gain (both already agree on the name).
    const kvsStreamArn = cdk.Arn.format(
      {
        service: "kinesisvideo",
        resource: "stream",
        resourceName: `${deviceId}-camera/*`,
      },
      this
    );

    role.addToPolicy(
      new iam.PolicyStatement({
        actions: [
          "kinesisvideo:DescribeStream",
          "kinesisvideo:GetDataEndpoint",
          "kinesisvideo:PutMedia",
        ],
        resources: [kvsStreamArn],
      })
    );

    // The awslogs driver creates its own stream inside the group above and
    // describes it to resume a sequence token.
    //
    // One ARN, not two. AWS::Logs::LogGroup's Arn attribute already ends in `:*`
    // (it is documented that way), so logGroupArn covers every stream in the
    // group; the second entry this used to carry appended another `:*` and
    // resolved to `…:log-group:/go2/ec2:*:*`, which matches nothing.
    role.addToPolicy(
      new iam.PolicyStatement({
        actions: ["logs:CreateLogStream", "logs:PutLogEvents", "logs:DescribeLogStreams"],
        resources: [logGroup.logGroupArn],
      })
    );

    // --- Boot: docker, a run script, and a systemd unit -------------------
    const userData = ec2.UserData.forLinux();

    // Every CloudFormation-resolved value is confined to this one file, so the
    // run script below can stay static, greppable, and runnable by hand while
    // debugging.
    userData.addCommands(
      "set -euxo pipefail",
      // The GPU AMI already has Docker and the NVIDIA Container Toolkit, so this
      // is a no-op there. Kept conditional rather than deleted so a plain AL2023
      // AMI (CPU-only fallback) still boots — and NOT unconditional, because
      // reinstalling over the AMI's Docker can shuffle the toolkit's runtime
      // registration and break --gpus.
      "command -v docker >/dev/null || dnf install -y docker",
      "systemctl enable --now docker",
      "",
      "cat >/etc/go2.env <<GO2ENV",
      `AWS_REGION=${this.region}`,
      `IMAGE=${repo.repositoryUri}:${imageTag.valueAsString}`,
      `LOG_GROUP=${logGroup.logGroupName}`,
      `SECRET_NAME=${unitreeSecretName.valueAsString}`,
      `KVS_STREAM_NAME=${deviceId}-camera`,
      // Solution attribution, resolved from this template's Solution mapping and
      // passed into the container below. go2_robot_sdk.aws_solution reads it and
      // hands it to botocore as user_agent_extra, so the KVS uploads and the
      // Secrets Manager fetch this instance makes carry the solution string.
      `USER_AGENT_STRING=${solutionUserAgent(this)}`,
      "GO2ENV",
      "chmod 600 /etc/go2.env",
      "",
      // Seeded only if absent, so a `cdk deploy` (or the instance replacement a
      // user-data change causes) does not silently drag the robot back to remote
      // mode while a laptop bridge is driving it.
      "test -f /etc/go2.mode || echo remote >/etc/go2.mode"
    );

    // Quoted heredoc ('GO2RUN') — nothing in here is expanded at boot, it is
    // written verbatim and evaluated when the unit starts.
    userData.addCommands(
      "cat >/usr/local/bin/go2-run.sh <<'GO2RUN'",
      "#!/usr/bin/env bash",
      "# Pull the current image and run the ROS 2 stack in the foreground;",
      "# systemd owns the lifecycle. Safe to run by hand to debug a bad start.",
      "set -euo pipefail",
      "source /etc/go2.env",
      "export AWS_DEFAULT_REGION=\"$AWS_REGION\"",
      "",
      "# Fail here, loudly, rather than 200 lines into a ROS launch. Without a",
      "# working driver `docker run --gpus all` errors with a runtime message that",
      "# says nothing about drivers, and coco_device:=cuda would raise deep inside",
      "# torch. Both are much harder to read than this line in `journalctl -u go2`.",
      "if ! nvidia-smi >/dev/null 2>&1; then",
      "  echo \"FATAL: no usable NVIDIA GPU. Is this a g-family instance on the\" >&2",
      "  echo \"       Deep Learning Base OSS NVIDIA Driver AMI? nvidia-smi says:\" >&2",
      "  nvidia-smi >&2 || true",
      "  exit 1",
      "fi",
      "nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader",
      "",
      "aws ecr get-login-password --region \"$AWS_REGION\" \\",
      "  | docker login --username AWS --password-stdin \"${IMAGE%%/*}\"",
      "",
      "# A no-op after the first boot: the layers are already on the volume.",
      "# This single line is the difference between a 40-second start here and a",
      "# 2-4 minute cold pull.",
      "docker pull \"$IMAGE\"",
      "",
      "# Which end owns the robot link. Read at every unit start from a file that",
      "# survives stop/start, so switching costs a container restart (seconds)",
      "# instead of an instance replacement and a cold 14 GB pull:",
      "#",
      "#   remote  this instance connects via Unitree's TURN server (the default)",
      "#   bridge  something else owns the dog — a laptop on the robot's own AP",
      "#           running scripts/go2_ap_bridge.py — and feeds this ROS graph",
      "#           through foxglove_bridge's clientPublish\n",
      "# /etc, not /run: /run is tmpfs and would silently revert to remote on the",
      "# next boot, which is the failure this file exists to prevent.",
      "MODE=\"$(cat /etc/go2.mode 2>/dev/null || echo remote)\"",
      "case \"$MODE\" in remote|bridge) ;; *)",
      "  echo \"FATAL: /etc/go2.mode is '$MODE'; expected 'remote' or 'bridge'\" >&2",
      "  exit 1 ;; esac",
      "echo \"go2: robot link mode = $MODE\"",
      "",
      "install -d -m 700 /run/go2",
      "umask 077",
      ": >/run/go2/container.env",
      "",
      "# In bridge mode the Unitree credentials are NOT fetched, on purpose.",
      "#",
      "# CONN_TYPE=bridge already stops go2_connection from taking its remote",
      "# branch, but leaving the credentials out is the stronger guarantee: with no",
      "# UNITREE_EMAIL/PASSWORD in the container there is no way to log in to",
      "# global-robot-api.unitree.com even if something were misconfigured, so this",
      "# instance cannot contribute to the EdgeOne WAF blocks or a reconnect storm",
      "# while the dog is on its own AP. Verifiable from outside: `docker exec go2",
      "# env | grep UNITREE` comes back empty.",
      "if [ \"$MODE\" = remote ]; then",
      "  # The entire robot link — email, password, serial — comes from the one",
      "  # secret and goes STRAIGHT into the env file: never through argv (`ps` is",
      "  # world-readable) and never through a shell variable. /run is tmpfs and",
      "  # umask makes the file 0600, so none of it touches disk.",
      "  aws secretsmanager get-secret-value --secret-id \"$SECRET_NAME\" \\",
      "    --region \"$AWS_REGION\" --query SecretString --output text \\",
      "    | python3 -c 'import json,sys",
      "s = json.load(sys.stdin)",
      "keys = {\"email\": \"UNITREE_EMAIL\", \"password\": \"UNITREE_PASSWORD\", \"serial\": \"UNITREE_SERIAL\"}",
      "missing = [k for k in keys if not s.get(k)]",
      "if missing:",
      "    sys.exit(\"secret %s is missing key(s): %s\" % (sys.argv[1], \", \".join(missing)))",
      "for k, var in keys.items():",
      "    print(var + \"=\" + s[k])' \"$SECRET_NAME\" >>/run/go2/container.env",
      "else",
      "  echo \"go2: bridge mode - skipping the Unitree secret fetch entirely\"",
      "fi",
      "",
      "# The rest of the container environment. This is the ONLY definition of the",
      "# cloud runtime config — there is deliberately no compose-file counterpart.",
      "# docker/docker-compose.mac.yml is a genuinely different setup (LAN webrtc,",
      "# port mapping, CPU detector), not a mirror of this, so pretending otherwise",
      "# only created a file that drifted.",
      "cat >>/run/go2/container.env <<ENVFILE",
      "CONN_TYPE=$MODE",
      "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
      "KVS_STREAM_NAME=$KVS_STREAM_NAME",
      "KVS_IMAGE_TOPIC=/annotated_image",
      "KVS_SEGMENT_SEC=2",
      "AWS_DEFAULT_REGION=$AWS_REGION",
      // Sourced from /etc/go2.env like AWS_REGION above, so the solution string
      // stays a single CloudFormation-resolved value in one file.
      "USER_AGENT_STRING=$USER_AGENT_STRING",
      "ENVFILE",
      "",
      "# A container left behind by an unclean stop would take the name.",
      "docker rm -f go2 >/dev/null 2>&1 || true",
      "",
      "# --network host: the bridge lands on the instance's own 8765 (which the",
      "# SSM port-forward targets), the DDS graph is free to leave the container,",
      "# and boto3 reaches IMDS without the bridge-network hop-limit problem.",
      "exec docker run --rm --name go2 \\",
      "  --network host \\",
      // Exposes the T4 to the container via the NVIDIA Container Toolkit. The
      // image already carries a CUDA-enabled torch — requirements.txt installs
      // plain `torch` from PyPI, whose linux x86_64 wheel bundles CUDA, which is a
      // good part of why the image is 14 GB. Nothing in the image needed changing.
      "  --gpus all \\",
      "  --env-file /run/go2/container.env \\",
      "  --log-driver=awslogs \\",
      "  --log-opt awslogs-region=\"$AWS_REGION\" \\",
      "  --log-opt awslogs-group=\"$LOG_GROUP\" \\",
      "  --log-opt awslogs-stream=\"go2/robot\" \\",
      "  \"$IMAGE\" \\",
      "  ros2 launch go2_robot_sdk robot.launch.py \\",
      "    rviz2:=false nav2:=false slam:=false \\",
      "    foxglove:=true joystick:=false teleop:=false \\",
      // coco_device:=cuda is the line that actually moves inference onto the GPU.
      // coco_detector_node has always taken a `device` parameter and used it
      // correctly (.to(device) on the model, device= on the input tensor), but it
      // defaults to 'cpu' and robot.launch.py did not plumb it through — so the
      // GPU would have sat idle with everything else here in place.
      //
      // coco_model:=best is now affordable: 46.7 mAP against 'balanced''s 37.0.
      // On CPU it was ~3900 ms/frame (~0.25 fps) and unusable for the greeter and
      // tracking loops; the T4 is what makes it the sensible default.
      "    coco:=true coco_device:=cuda coco_model:=best \\",
      "    compressed:=true kvs:=true",
      "GO2RUN",
      "chmod 755 /usr/local/bin/go2-run.sh"
    );

    // Restart=on-failure with a start limit, NOT `always`: an instance left
    // running while the robot is switched off becomes a reconnect loop pointed
    // at the vendor's cloud, which is how you get the source IP blocked. Five
    // failures in ten minutes and systemd gives up instead of hammering it.
    // (`make stop-ec2` remains the real answer — this is the backstop.)
    userData.addCommands(
      "cat >/etc/systemd/system/go2.service <<'GO2UNIT'",
      "[Unit]",
      "Description=Go2 ROS 2 stack (container)",
      "Requires=docker.service",
      "After=docker.service network-online.target",
      "StartLimitIntervalSec=600",
      "StartLimitBurst=5",
      "",
      "[Service]",
      "Type=simple",
      "ExecStart=/usr/local/bin/go2-run.sh",
      "ExecStop=/usr/bin/docker stop -t 30 go2",
      "Restart=on-failure",
      "RestartSec=30",
      "",
      "[Install]",
      "WantedBy=multi-user.target",
      "GO2UNIT",
      "systemctl daemon-reload",
      // enable --now, so `deploy` gives a running robot and every subsequent
      // `make start-ec2` (an instance start) brings it back with no extra step.
      "systemctl enable --now go2.service"
    );

    const instance = new ec2.Instance(this, "Go2Robot", {
      vpc,
      // Pinned to the one subnet that contains robotPrivateIp — not just "a
      // public subnet" — because the address below has to be inside its range.
      vpcSubnets: { subnets: [props.robotSubnet] },
      privateIpAddress: props.robotPrivateIp,
      instanceType: new ec2.InstanceType(instanceType),
      machineImage: ec2.MachineImage.fromSsmParameter(gpuAmiParameter, {
        // fromSsmParameter can't infer the OS from a parameter it resolves at
        // deploy time, so it has to be stated; without it CDK assumes Windows and
        // renders the user data as PowerShell.
        os: ec2.OperatingSystemType.LINUX,
      }),
      securityGroup: sg,
      role,
      userData,
      // Without a public IP this instance has no route out (its subnet routes to
      // the IGW, not the NAT), and SSM would never register it. It also gives a
      // fresh source IP on every start, which is the Unitree WAF escape hatch.
      associatePublicIpAddress: true,
      requireImdsv2: true,
      // 1-minute metrics instead of 5. Nearly free for this instance in
      // particular — detailed monitoring bills per metric-hour and a stopped
      // instance emits nothing, so at ~4h of use a day it is cents — and 5-minute
      // granularity is genuinely too coarse for the thing being watched: a
      // detector saturating the T4 or the container thrashing shows up and is
      // gone inside one datapoint.
      detailedMonitoring: true,
      blockDevices: [
        {
          deviceName: "/dev/xvda",
          volume: ec2.BlockDeviceVolume.ebs(volumeSizeGb, {
            volumeType: ec2.EbsDeviceVolumeType.GP3,
            encrypted: true,
            deleteOnTermination: true,
          }),
        },
      ],
      // cloud-init runs once per instance, so a user-data edit that did not
      // replace would leave the box running config the template no longer
      // describes. Replacing keeps them honest, at the cost of one cold 14 GB
      // pull onto the new volume — which is why routine image rolls go through
      // `make restart-ec2` (no template change, cache intact) and not through
      // bumping ImageTag here.
      userDataCausesReplacement: true,
    });

    // --- Auto-stop watchdog -----------------------------------------------
    // Every 5 minutes: if the instance has been running longer than its budget,
    // revoke any temporary internet rule and stop it. See the class doc above for
    // why this is a Lambda and not a systemd timer on the box.
    //
    // NOT in the VPC, on purpose: it calls the EC2 control-plane API, so a VPC
    // Lambda would need either the NAT (which this path exists to avoid depending
    // on) or an interface endpoint. Outside the VPC it has AWS-managed egress for
    // free and can still stop the instance when the VPC itself is the problem.
    const autoStopLogs = new logs.LogGroup(this, "AutoStopLogGroup", {
      retention: logs.RetentionDays.ONE_WEEK,
      removalPolicy: cdk.RemovalPolicy.DESTROY,
    });

    // Own role, for the same reason SceneDescriber has one: CDK's default Lambda
    // role attaches AWSLambdaBasicExecutionRole, whose log grants are on Resource
    // "*" (AwsSolutions-IAM4). The log group is declared above, so the grant can
    // name it and logs:CreateLogGroup is not needed.
    const autoStopRole = new iam.Role(this, "AutoStopRole", {
      assumedBy: new iam.ServicePrincipal("lambda.amazonaws.com"),
      description: "Go2 auto-stop watchdog - own log group, stop this instance only",
    });
    autoStopLogs.grantWrite(autoStopRole);

    const autoStop = new lambda.Function(this, "AutoStopFn", {
      // Latest Python runtime (AwsSolutions-L1). boto3 is the only dependency and
      // every runtime ships it, so this moves forward with no code change.
      runtime: lambda.Runtime.PYTHON_3_14,
      role: autoStopRole,
      handler: "index.handler",
      timeout: cdk.Duration.seconds(30),
      logGroup: autoStopLogs,
      description: `Stops ${deviceId} after ${autoStopMinutes} min of uptime (tag-overridable)`,
      environment: {
        INSTANCE_ID: instance.instanceId,
        DEFAULT_MINUTES: String(autoStopMinutes),
        // Lets the revoke below tell the one CDK-managed in-VPC rule apart from
        // the temporary /32s `ec2-ctl.sh open` adds, without a describe-vpcs call.
        SECURITY_GROUP_ID: sg.securityGroupId,
        VPC_CIDR: vpc.vpcCidrBlock,
        BRIDGE_PORT: "8765",
        // Solution attribution for this function's EC2 API calls. Read with
        // os.environ[...] below rather than .get(): it is always set here, and a
        // missing value should fail loudly at import rather than quietly drop
        // the watchdog's calls out of the solution's usage reporting.
        USER_AGENT_STRING: solutionUserAgent(this),
      },
      // Inline rather than an asset: it is short, it has no dependencies beyond
      // boto3, and keeping it in this file means the policy, the schedule, and the
      // logic that relies on both are read together.
      code: lambda.Code.fromInline(`
import datetime
import os

import boto3
from botocore.config import Config

# AWSSOLUTION/<id>/<version>, from this stack's Solution mapping. Appended to the
# SDK's User-Agent so this function's EC2 calls are attributable to the solution.
ec2 = boto3.client(
    "ec2", config=Config(user_agent_extra=os.environ["USER_AGENT_STRING"])
)

INSTANCE_ID = os.environ["INSTANCE_ID"]
DEFAULT_MINUTES = int(os.environ["DEFAULT_MINUTES"])
SECURITY_GROUP_ID = os.environ["SECURITY_GROUP_ID"]
VPC_CIDR = os.environ["VPC_CIDR"]
BRIDGE_PORT = int(os.environ["BRIDGE_PORT"])

# Every knob is an instance tag, so changing any of them is one API call from a
# laptop rather than a stack deploy. scripts/ec2-ctl.sh writes all three.
ARMED_TAG = "go2:autostop"            # "off" disarms the watchdog entirely
BUDGET_TAG = "go2:autostop-minutes"   # per-instance budget, overrides the default
UNTIL_TAG = "go2:autostop-until"      # ISO-8601 UTC reprieve for one session


def close_direct():
    """Revoke ingress on the bridge port from anything outside the VPC.

    Mirrors close_direct() in scripts/ec2-ctl.sh, and exists so an auto-stop
    upholds the same invariant its 'stop' verb does: a session cannot leave the
    unauthenticated bridge reachable from the internet. Nothing is listening once
    the instance is stopped, but a rule aimed at a stale address would come back
    to life the moment someone starts the instance from the console.

    Fail-soft on purpose: stopping the instance is the job, and a revoke problem
    must never be the reason a $0.526/hr box keeps running.
    """
    try:
        rules = ec2.describe_security_group_rules(
            Filters=[{"Name": "group-id", "Values": [SECURITY_GROUP_ID]}]
        )["SecurityGroupRules"]
        ids = [
            r["SecurityGroupRuleId"]
            for r in rules
            if not r["IsEgress"]
            and r.get("FromPort") == BRIDGE_PORT
            and r.get("CidrIpv4") not in (None, VPC_CIDR)
        ]
        if not ids:
            return
        print("revoking external rule(s) on %d: %s" % (BRIDGE_PORT, ",".join(ids)))
        ec2.revoke_security_group_ingress(
            GroupId=SECURITY_GROUP_ID, SecurityGroupRuleIds=ids
        )
    except Exception as exc:  # noqa: BLE001 - see the docstring
        print("WARNING: could not revoke external access: %s" % exc)


def handler(event, context):
    inst = ec2.describe_instances(InstanceIds=[INSTANCE_ID])[
        "Reservations"
    ][0]["Instances"][0]
    state = inst["State"]["Name"]
    if state != "running":
        print("instance is %s - nothing to do" % state)
        return {"state": state, "stopped": False}

    tags = {t["Key"]: t["Value"] for t in inst.get("Tags", [])}

    armed = tags.get(ARMED_TAG, "on").strip().lower()
    if armed in ("off", "false", "no", "0"):
        print("%s=%s - disarmed, leaving it running" % (ARMED_TAG, armed))
        return {"state": state, "stopped": False, "disarmed": True}

    minutes = DEFAULT_MINUTES
    raw = tags.get(BUDGET_TAG)
    if raw:
        try:
            minutes = int(raw.strip())
        except ValueError:
            print("ignoring unparseable %s=%r, using %d" % (BUDGET_TAG, raw, minutes))

    # LaunchTime is the last START for an EBS-backed instance, not the original
    # launch, so this is a per-session clock that every stop/start resets and
    # nothing has to remember across one.
    now = datetime.datetime.now(datetime.timezone.utc)
    uptime_min = (now - inst["LaunchTime"]).total_seconds() / 60.0
    deadline = inst["LaunchTime"] + datetime.timedelta(minutes=minutes)

    until = tags.get(UNTIL_TAG)
    if until:
        try:
            asked = datetime.datetime.fromisoformat(until.strip().replace("Z", "+00:00"))
            if asked.tzinfo is None:
                asked = asked.replace(tzinfo=datetime.timezone.utc)
            # Only ever later, never earlier: an extension cannot become an early
            # kill. It is also self-expiring — a tag left over from a previous
            # session is in the past, hence not later than the deadline, hence
            # ignored, so there is nothing to clean up.
            if asked > deadline:
                print("reprieve until %s (%s)" % (asked.isoformat(), UNTIL_TAG))
                deadline = asked
        except ValueError:
            print("ignoring unparseable %s=%r" % (UNTIL_TAG, until))

    left_min = (deadline - now).total_seconds() / 60.0
    if left_min > 0:
        # One line per poll, which makes 'aws logs tail' the answer to "how long
        # have I got?" as well as "why did it stop?".
        print(
            "up %.0f min, budget %d min, %.0f min left (deadline %s)"
            % (uptime_min, minutes, left_min, deadline.isoformat(timespec="seconds"))
        )
        return {
            "state": state,
            "stopped": False,
            "uptimeMinutes": round(uptime_min),
            "minutesLeft": round(left_min),
        }

    print(
        "STOPPING %s: up %.0f min, %.0f min past its %d min budget"
        % (INSTANCE_ID, uptime_min, -left_min, minutes)
    )
    close_direct()
    # No force flag: the ACPI shutdown runs go2.service's ExecStop, so the
    # container gets a clean SIGTERM and the WebRTC session closes properly
    # instead of timing out on the robot's side.
    ec2.stop_instances(InstanceIds=[INSTANCE_ID])
    return {"state": state, "stopped": True, "uptimeMinutes": round(uptime_min)}
`),
    });

    // Scoped to this one instance. DescribeInstances and DescribeSecurityGroupRules
    // have no resource-level permissions at all, so they are "*" by necessity —
    // both are read-only. The two mutating calls are not.
    autoStop.addToRolePolicy(
      new iam.PolicyStatement({
        actions: ["ec2:DescribeInstances", "ec2:DescribeSecurityGroupRules"],
        resources: ["*"],
      })
    );
    autoStop.addToRolePolicy(
      new iam.PolicyStatement({
        actions: ["ec2:StopInstances"],
        resources: [
          cdk.Arn.format(
            {
              service: "ec2",
              resource: "instance",
              resourceName: instance.instanceId,
            },
            this
          ),
        ],
      })
    );
    autoStop.addToRolePolicy(
      new iam.PolicyStatement({
        actions: ["ec2:RevokeSecurityGroupIngress"],
        resources: [
          cdk.Arn.format(
            {
              service: "ec2",
              resource: "security-group",
              resourceName: sg.securityGroupId,
            },
            this
          ),
        ],
      })
    );

    // 5 minutes: the granularity of the stop (so the real budget is 4h00-4h05) and
    // ~8.6k invocations a month, which is inside the Lambda free tier and pennies
    // outside it. A 1-minute rate would be 5x the invocations to save 4 minutes of
    // an instance that is by then already forgotten.
    new events.Rule(this, "AutoStopSchedule", {
      description: `Poll ${deviceId} uptime and stop it past its budget`,
      schedule: events.Schedule.rate(cdk.Duration.minutes(5)),
      targets: [new targets.LambdaFunction(autoStop)],
    });

    // --- cdk-nag: what is deliberate here ----------------------------------
    NagSuppressions.addResourceSuppressions(
      role,
      [
        {
          id: "AwsSolutions-IAM4",
          reason:
            "AmazonSSMManagedInstanceCore is the AWS-documented prerequisite for " +
            "Session Manager, and SSM is the ONLY way into this instance (there " +
            "is no inbound rule for a human and no key pair). A hand-rolled " +
            "equivalent would be strictly worse: ssmmessages:*, ec2messages:* and " +
            "ssm:UpdateInstanceInformation support no resource-level permissions, " +
            "so it would be the same access written as a wildcard we maintain.",
          appliesTo: [
            "Policy::arn:<AWS::Partition>:iam::aws:policy/AmazonSSMManagedInstanceCore",
          ],
        },
        {
          id: "AwsSolutions-IAM5",
          reason:
            "Stream ARNs in Kinesis Video Streams end in a service-assigned " +
            "creation timestamp, so the exact ARN is unknowable at synth. Scoped " +
            "to this robot's stream name; the wildcard covers only that segment.",
          // Regex rather than a literal: cdk-nag renders the finding with the
          // region and account resolved, so a template string built from
          // kvsStreamArn (which still holds an AWS::Partition token) would not
          // match and the suppression would silently do nothing.
          appliesTo: [
            {
              regex:
                "/^Resource::arn:<AWS::Partition>:kinesisvideo:.*:stream\\/.*-camera\\/\\*$/g",
            },
          ],
        },
        {
          id: "AwsSolutions-IAM5",
          reason:
            "ecr:GetAuthorizationToken supports no resource-level permissions — " +
            "'*' is its only valid form. Written by repo.grantPull; the pull " +
            "actions it accompanies ARE scoped to the go2-ros2-sdk repository.",
          appliesTo: ["Resource::*"],
        },
      ],
      true
    );

    NagSuppressions.addResourceSuppressions(
      instance,
      [
        {
          id: "AwsSolutions-EC29",
          reason:
            "Termination protection would BREAK this stack, not harden it. " +
            "userDataCausesReplacement is true by design (cloud-init runs once, so " +
            "a user-data edit must replace the instance or the box silently keeps " +
            "running config the template no longer describes) and CloudFormation " +
            "cannot replace an instance it is not allowed to terminate — the " +
            "deploy would fail, as would `cdk destroy`. The volume is the only " +
            "state and it is reproducible: a fresh instance re-pulls the image.",
        },
      ],
      true
    );

    // Outside the VPC on purpose, and the reasoning is in the comment above the
    // construct: it calls the EC2 control plane, so in a subnet it would need
    // either the NAT this architecture exists to avoid or another paid interface
    // endpoint — and, more to the point, it must still be able to stop the
    // instance when the VPC or the instance itself is the thing that is broken.
    // It has no URL and no public trigger; EventBridge is the only caller.
    NagSuppressions.addResourceSuppressions(autoStop, [
      {
        id: "Prototype Security Nag Pack-LambdaInsideVPC",
        reason:
          "Calls the EC2 control plane only. In-VPC it would need a NAT or a " +
          "paid endpoint, and it must keep working when the VPC is the fault. " +
          "EventBridge-triggered only: no function URL, no public trigger.",
      },
    ]);

    NagSuppressions.addResourceSuppressions(
      autoStopRole,
      [
        {
          id: "AwsSolutions-IAM5",
          reason:
            "ec2:DescribeInstances and ec2:DescribeSecurityGroupRules support no " +
            "resource-level permissions at all — '*' is the only valid form. Both " +
            "are read-only; the two mutating calls (StopInstances, " +
            "RevokeSecurityGroupIngress) are pinned to this instance and this " +
            "security group by ARN.",
          appliesTo: ["Resource::*"],
        },
      ],
      true
    );

    // Same token-valued-CIDR false positive as the VPC endpoint SGs: the one
    // ingress rule here is 8765 from vpc.vpcCidrBlock, an Fn::GetAtt that EC23
    // cannot resolve at synth. The rule stays armed everywhere else.
    NagSuppressions.addResourceSuppressions(sg, [
      {
        id: "CdkNagValidationFailure",
        reason:
          "Ingress is 8765 from vpc.vpcCidrBlock, which resolves to " +
          "Fn::GetAtt(vpc, CidrBlock) at deploy time, so AwsSolutions-EC23 " +
          "cannot evaluate it at synth. A VPC's own CIDR is never 0.0.0.0/0.",
      },
    ]);

    // --- Outputs ----------------------------------------------------------
    // InstanceId is the only value the scripts need: it is simultaneously the
    // start/stop target and the SSM tunnel target. That is the whole reason this
    // stack has no NLB and no jumpbox — one resource plays every role.
    new cdk.CfnOutput(this, "InstanceId", {
      value: instance.instanceId,
      description:
        "Robot instance — target for scripts/ec2-ctl.sh and tunnel.sh (GO2_INSTANCE)",
    });

    new cdk.CfnOutput(this, "BridgeHostInVpc", {
      value: `${props.robotPrivateIp}:8765`,
      description:
        "Where in-VPC clients (the AgentCore runtime) reach the foxglove bridge",
    });

    new cdk.CfnOutput(this, "EcrRepoUri", {
      value: repo.repositoryUri,
      description: "Push the container image here (make push-image)",
    });

    new cdk.CfnOutput(this, "LogGroupName", {
      value: logGroup.logGroupName,
      description: "Container stdout — `aws logs tail /go2/ec2 --follow`",
    });

    new cdk.CfnOutput(this, "AutoStopMinutes", {
      value: String(autoStopMinutes),
      description:
        "Uptime budget before the watchdog stops the instance (override per-session: make extend-ec2)",
    });

    new cdk.CfnOutput(this, "AutoStopLogGroupName", {
      value: autoStopLogs.logGroupName,
      description:
        "Watchdog decisions, one line per 5-minute poll — includes how long is left",
    });
  }
}
