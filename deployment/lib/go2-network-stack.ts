// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: MIT-0
import * as cdk from "aws-cdk-lib";
import * as ec2 from "aws-cdk-lib/aws-ec2";
import * as logs from "aws-cdk-lib/aws-logs";
import { NagSuppressions } from "cdk-nag";
import { Construct } from "constructs";
import { applySolutionMetadata } from "./solution";

/**
 * Fixed private address for the robot instance.
 *
 * Pinning it is what lets Go2AgentCoreStack point at the robot with a plain
 * string instead of a CloudFormation import. That matters more than it looks:
 * an `Fn::ImportValue` on the instance's generated private IP would make the
 * value un-updatable while the agent imports it, so replacing the instance
 * (which a user-data change does by design) would fail with "Cannot update an
 * export in use". A literal on both sides has no such coupling — the two stacks
 * agree on an address, not on a resource.
 *
 * Must fall inside the robot subnet's CIDR (see ROBOT_AZ_INDEX and
 * ROBOT_SUBNET_CIDR); the stack asserts that below rather than trusting it.
 */
export const ROBOT_PRIVATE_IP = "10.0.1.10";

/**
 * Which AZ the robot lives in, as an index into the VPC's AZ list — 0 is the
 * first (us-east-1a), 1 the second (us-east-1b).
 *
 * ONE constant for two placements on purpose: the robot's public subnet and the
 * interface endpoints below both use it, which is what keeps the endpoints in the
 * robot's AZ (asserted below). Changing it means changing ROBOT_PRIVATE_IP and
 * ROBOT_SUBNET_CIDR with it, since each public subnet has its own /24 — and it
 * REPLACES the instance, because an AZ move cannot be an in-place update. That
 * costs one cold 14 GB image pull on first boot; nothing on the root volume is
 * anything but cache.
 *
 * Moving it again needs THREE deploys, not one, because CDK exports only the
 * subnet the consumer imports: the deploy that adds the new export also deletes
 * the old one, which CloudFormation refuses while Go2Ec2Stack still imports it —
 * and that stack cannot switch until the new export exists. Break the deadlock by
 * retaining the outgoing name for one round trip:
 * `this.exportValue(this.vpc.publicSubnets[<old index>].subnetId)`, then deploy
 * network, deploy ec2, drop the line, deploy network again.
 */
const ROBOT_AZ_INDEX = 1;

/** CIDR the robot subnet is expected to get. Asserted, not assumed. */
const ROBOT_SUBNET_CIDR = "10.0.1.0/24";

/**
 * The one VPC everything Go2-related runs in.
 *
 * Replaces the per-stack VPCs that Go2FargateStack and Go2Ec2Stack used to build
 * for themselves — which, besides being duplicative, both used 10.0.0.0/16 and so
 * could never have been peered if anything had needed to talk across them. The
 * hosted agent could only ever reach one of them, which is what forced the issue.
 *
 * Layout, and the reasoning behind each half:
 *
 *   public   10.0.0.0/24, 10.0.1.0/24   the robot instance
 *   isolated 10.0.2.0/24, 10.0.3.0/24   the AgentCore runtime's ENIs
 *
 * The robot sits in a PUBLIC subnet with its own public IP, deliberately. Two
 * reasons:
 *
 *   1. Egress via the internet gateway is free, and the robot's WebRTC media
 *      plus ECR pulls are the only real traffic here.
 *   2. A public-subnet instance gets a FRESH public IP on every start. Unitree's
 *      cloud WAF blocks by source IP after a reconnect storm, and a stop/start is
 *      the only reliable way to shake it. A stable egress address makes the block
 *      stick — that was a real, recurring failure on the Fargate path.
 *
 * THERE IS NO NAT GATEWAY. The AgentCore subnets are ISOLATED and reach AWS
 * through four single-AZ interface endpoints instead — ~$29/month against the
 * NAT's ~$33. A thin margin; see the kvs caveat at the bottom before assuming it
 * is a good trade.
 *
 * Everything the runtime needs over these ENIs, each one established by watching
 * it FAIL first:
 *
 *   - ECR image pull (ecr.api + ecr.dkr + the s3 gateway for layers). Without
 *     these the runtime cannot provision at all: HTTP 424 (Failed Dependency) to
 *     the caller and an EMPTY log group, because the container never boots.
 *   - Nova Sonic (bedrock-runtime). Without this the container boots and the
 *     bridge connects, then the session dies ~5 s later with
 *     `AwsCrtError: AWS_IO_SOCKET_TIMEOUT` from nova_sonic.py's stream setup.
 *   - CloudWatch Logs (logs). Also carries X-Ray, since tracingEnabled uses
 *     CloudWatch Logs as the trace segment destination.
 *   - The foxglove bridge at ROBOT_PRIVATE_IP:8765: in-VPC, covered by `local`.
 *
 * BEWARE the trap that cost us an afternoon: a version that is already running
 * keeps working after you remove egress, because the image is cached on a warm
 * host. It only breaks on the next cold provision — i.e. the next runtime
 * version, which can be hours later. Never conclude "no NAT needed" from a
 * session that was already up; force a new version and re-test.
 *
 * A PUBLIC subnet is NOT an alternative to any of this. AgentCore ENIs never get
 * a public IP and an IGW only translates for ENIs that have one, so a public
 * subnet yields a 0.0.0.0/0 route that silently blackholes. Tested: also 424.
 *
 * The one thing no endpoint can fix is Kinesis Video Streams: it has NO
 * PrivateLink endpoint in us-east-1 (all of kinesisvideo, -media, -signaling are
 * InvalidServiceName), so anything in these subnets that touches KVS needs a real
 * NAT. Nothing does anymore — the kvs_* voice tools that did were deleted for
 * exactly this reason, and the robot uploads to KVS from a public subnet via the
 * IGW. describe_scene / compare_scenes call Bedrock and were never affected.
 *
 * Nothing here is looked up by tag. The old stacks discovered each other that way
 * to stay independently deployable — specifically so redeploying the agent could
 * not reset the Fargate service's desiredCount and stop a robot mid-session. With
 * Fargate gone there is no desiredCount to reset, so the stacks take this VPC as
 * a plain construct prop and the whole lookup dance disappears.
 */
export class Go2NetworkStack extends cdk.Stack {
  public readonly vpc: ec2.Vpc;
  /** The specific subnet ROBOT_PRIVATE_IP lives in. */
  public readonly robotSubnet: ec2.ISubnet;

  constructor(scope: Construct, id: string, props?: cdk.StackProps) {
    super(scope, id, props);

    // No user-agent mapping here: this stack deploys no code of ours, so there
    // is nothing to hand the string to. Description only.
    applySolutionMetadata(
      this,
      "Unitree Go2 AWS robotics demo - the shared VPC the robot instance and " +
        "the hosted voice agent both live in (no NAT; interface endpoints only)"
    );

    this.vpc = new ec2.Vpc(this, "Go2Vpc", {
      ipAddresses: ec2.IpAddresses.cidr("10.0.0.0/16"),
      maxAzs: 2,
      // No NAT. Nothing in this VPC needs one: the robot egresses via the IGW
      // from its public subnet, and the AgentCore ENIs below only ever dial an
      // in-VPC address. See the class doc for what was measured.
      natGateways: 0,
      subnetConfiguration: [
        { name: "public", subnetType: ec2.SubnetType.PUBLIC, cidrMask: 24 },
        {
          // Keep the group NAME "private" even though the type is now isolated:
          // CDK derives subnet logical IDs from the name, so renaming it would
          // REPLACE both subnets, and AgentCore's ENIs (plus the pinned
          // ROBOT_SUBNET_CIDR assertion below) depend on the current allocation.
          name: "private",
          // ISOLATED, not PRIVATE_WITH_EGRESS — with natGateways: 0 the latter
          // fails at synth, because CDK has no egress target to route it to.
          subnetType: ec2.SubnetType.PRIVATE_ISOLATED,
          cidrMask: 24,
        },
      ],
    });

    // --- Flow logs (AwsSolutions-VPC7) --------------------------------------
    // Genuinely useful here rather than box-ticking: the two hard failures this
    // VPC has produced — the AgentCore runtime silently blackholing to an absent
    // endpoint, and the robot SG dropping bridge traffic — both look identical
    // from the application side (a timeout) and both are unambiguous in a flow
    // log's REJECT/ACCEPT records.
    //
    // Destination is CloudWatch Logs, not S3, and the retention is short. An S3
    // destination would mean a bucket in this stack, and a bucket is the single
    // most nag-encumbered resource there is (encryption key, access logs, TLS
    // policy) for data nobody reads after the session it explains.
    const flowLogGroup = new logs.LogGroup(this, "FlowLogGroup", {
      retention: logs.RetentionDays.ONE_WEEK,
      removalPolicy: cdk.RemovalPolicy.DESTROY,
    });

    this.vpc.addFlowLog("FlowLog", {
      destination: ec2.FlowLogDestination.toCloudWatchLogs(flowLogGroup),
      trafficType: ec2.FlowLogTrafficType.ALL,
    });

    // ECR keeps image LAYER blobs in S3, so this carries the bulk of the bytes in
    // an image pull. A gateway endpoint has no hourly fee and only adds a
    // prefix-list route, so it is free and load-bearing: the ecr.* interface
    // endpoints below handle the API calls, but without this the layers
    // themselves have nowhere to come from.
    this.vpc.addGatewayEndpoint("S3Endpoint", {
      service: ec2.GatewayVpcEndpointAwsService.S3,
    });

    // --- Interface endpoints: what replaced the NAT -------------------------
    // Pinned to ONE subnet, not both. Interface endpoints bill per AZ
    // ($0.01/hr ≈ $7.30/mo each), so three of them in two AZs would be ~$44/mo —
    // MORE than the NAT they replace. In one AZ it is ~$22/mo. Private DNS is on,
    // so the runtime's ENI in the other AZ still resolves the regional name to
    // this endpoint and works; it just pays cross-AZ transfer on the pull. At one
    // image pull per cold provision that is pennies, and it is the whole reason
    // this is cheaper than a NAT.
    //
    // The AZ is the robot's, so the agent's hot path (bridge traffic to
    // ROBOT_PRIVATE_IP) stays in-AZ; the cold path (image pull) is the one that
    // may cross. Asserted below rather than assumed.
    const endpointSubnet = this.vpc.isolatedSubnets[ROBOT_AZ_INDEX];
    const endpointSubnets: ec2.SubnetSelection = { subnets: [endpointSubnet] };

    // The three the AgentCore docs require for a VPC-mode container agent with no
    // internet access: image pull (x2) and logs.
    this.vpc.addInterfaceEndpoint("EcrApiEndpoint", {
      service: ec2.InterfaceVpcEndpointAwsService.ECR,
      subnets: endpointSubnets,
    });
    this.vpc.addInterfaceEndpoint("EcrDockerEndpoint", {
      service: ec2.InterfaceVpcEndpointAwsService.ECR_DOCKER,
      subnets: endpointSubnets,
    });
    this.vpc.addInterfaceEndpoint("LogsEndpoint", {
      service: ec2.InterfaceVpcEndpointAwsService.CLOUDWATCH_LOGS,
      subnets: endpointSubnets,
    });

    // Nova Sonic. This one is NOT optional, despite an earlier belief that model
    // traffic was proxied service-side — it is not. Without this endpoint the
    // bridge connects, then the session dies ~5 s later with
    // `AwsCrtError: AWS_IO_SOCKET_TIMEOUT` out of nova_sonic.py's stream setup,
    // because InvokeModelWithBidirectionalStream has nowhere to go. One endpoint
    // covers both models: NOVA_SONIC_REGION and SCENE_REGION are both this region,
    // so Nova Sonic and the Claude behind describe_scene share it.
    this.vpc.addInterfaceEndpoint("BedrockRuntimeEndpoint", {
      service: ec2.InterfaceVpcEndpointAwsService.BEDROCK_RUNTIME,
      subnets: endpointSubnets,
    });

    // Each `addInterfaceEndpoint` builds a security group whose one ingress rule
    // is 443 from `vpc.vpcCidrBlock` — which synthesizes to Fn::GetAtt on the
    // VPC's CidrBlock, not a literal. AwsSolutions-EC23 (the "no 0.0.0.0/0
    // ingress" rule) cannot read a CIDR it has to resolve at deploy time, so it
    // raises CdkNagValidationFailure instead of a verdict. Suppress the failure,
    // NOT the rule: EC23 itself stays armed on every other security group, and
    // this one is a token by construction, so it can never be 0.0.0.0/0.
    for (const endpoint of [
      "EcrApiEndpoint",
      "EcrDockerEndpoint",
      "LogsEndpoint",
      "BedrockRuntimeEndpoint",
    ]) {
      NagSuppressions.addResourceSuppressions(
        this.vpc.node.findChild(endpoint),
        [
          {
            id: "CdkNagValidationFailure",
            reason:
              "Ingress is 443 from vpc.vpcCidrBlock, which resolves to " +
              "Fn::GetAtt(vpc, CidrBlock) at deploy time, so AwsSolutions-EC23 " +
              "cannot evaluate it at synth. A VPC's own CIDR is never 0.0.0.0/0.",
          },
        ],
        true
      );
    }

    this.robotSubnet = this.vpc.publicSubnets[ROBOT_AZ_INDEX];

    // The endpoints only stay in the robot's AZ by CDK allocating public and
    // isolated subnets in the same AZ order. If that ever changes, the pull hop
    // silently starts crossing AZs — cheap, but not what the comment above claims.
    if (
      !cdk.Token.isUnresolved(endpointSubnet.availabilityZone) &&
      !cdk.Token.isUnresolved(this.robotSubnet.availabilityZone) &&
      endpointSubnet.availabilityZone !== this.robotSubnet.availabilityZone
    ) {
      throw new Error(
        `Interface endpoints are in ${endpointSubnet.availabilityZone} but the ` +
          `robot is in ${this.robotSubnet.availabilityZone}. Both are indexed by ` +
          `ROBOT_AZ_INDEX, so CDK's public/isolated AZ ordering must have ` +
          `diverged — pick the isolated subnet matching the robot's AZ.`
      );
    }

    // ROBOT_PRIVATE_IP is a literal, and CDK's subnet CIDR allocation is what
    // makes it valid. Assert rather than hope: if a future change to maxAzs or
    // subnetConfiguration shifts the allocation, this fails at synth with a clear
    // message instead of at deploy with "invalid IP for subnet".
    const cidr = this.robotSubnet.ipv4CidrBlock;
    if (!cdk.Token.isUnresolved(cidr) && cidr !== ROBOT_SUBNET_CIDR) {
      throw new Error(
        `Robot subnet CIDR is ${cidr}, expected ${ROBOT_SUBNET_CIDR}. ` +
          `ROBOT_PRIVATE_IP (${ROBOT_PRIVATE_IP}) is pinned to that range — ` +
          `update both together in lib/go2-network-stack.ts.`
      );
    }

    new cdk.CfnOutput(this, "VpcId", {
      value: this.vpc.vpcId,
      description: "The shared Go2 VPC",
    });

    new cdk.CfnOutput(this, "RobotPrivateIp", {
      value: ROBOT_PRIVATE_IP,
      description: "Fixed private IP of the robot instance — AgentCore's ROS_BRIDGE_HOST",
    });
  }
}
