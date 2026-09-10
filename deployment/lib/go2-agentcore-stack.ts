// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: MIT-0
import * as cdk from "aws-cdk-lib";
import * as ec2 from "aws-cdk-lib/aws-ec2";
import * as iam from "aws-cdk-lib/aws-iam";
import * as agentcore from "aws-cdk-lib/aws-bedrockagentcore";
import { NagSuppressions } from "cdk-nag";
import { Construct } from "constructs";
import { applySolutionMetadata, solutionUserAgent } from "./solution";

export interface Go2AgentCoreStackProps extends cdk.StackProps {
  /**
   * No deviceId here: this stack no longer addresses the robot by name. It reaches
   * the ROS graph over bridgeHost and calls Bedrock directly. The KVS stream name
   * is agreed between Go2KVSStack (creates it) and Go2Ec2Stack (writes to it).
   */
  /** The shared VPC from Go2NetworkStack — the runtime's ENIs land here. */
  readonly vpc: ec2.IVpc;
  /**
   * Where the robot's foxglove bridge lives, as seen from inside the VPC. This is
   * Go2NetworkStack's ROBOT_PRIVATE_IP — a plain string, deliberately not a
   * reference to the instance. See the note there on why.
   */
  readonly bridgeHost: string;
  /** Port the bridge listens on. Default: 8765. */
  readonly bridgePort?: number;
  /**
   * The Bedrock Guardrail from Go2KVSStack, applied to the agent's own Bedrock
   * calls. Passed as plain strings for the same reason bridgeHost is — see the
   * note there — and OPTIONAL so this stack still deploys standalone against an
   * account where Go2KVSStack has not been created. Unset means the agent makes
   * unguarded model calls, which is why bin/app.ts always supplies it.
   */
  readonly guardrailId?: string;
  readonly guardrailVersion?: string;
  readonly guardrailArn?: string;
}

/**
 * Hosts the Go2 Nova Sonic voice agent on Amazon Bedrock AgentCore Runtime.
 *
 * This is the cloud counterpart to running `python -m voice_agent.main` on a
 * laptop. The agent — same system prompt, same tools, same Nova Sonic model —
 * moves into the cloud; only the microphone and speaker stay local:
 *
 *   laptop (voice_agent.agentcore_client)
 *     │  wss://bedrock-agentcore.<region>.amazonaws.com/runtimes/<arn>/ws
 *     │  (SigV4 pre-signed; access is IAM, no public endpoint of our own)
 *     ▼
 *   AgentCore Runtime  ── in the shared VPC ──►  10.0.1.10:8765
 *   (BidiAgent + Nova Sonic)                     (foxglove_bridge on the robot
 *                                                 instance, --network host)
 *
 * Because the runtime sits in the shared VPC's private subnets, the SSM tunnel
 * drops out of the agent's path entirely — `scripts/tunnel.sh` remains only for
 * viewing topics locally in Lichtblick or running the laptop-side agent.
 *
 * It talks to the robot at a FIXED PRIVATE IP, passed in as a plain string, with
 * no load balancer in between. Two things worth understanding about that:
 *
 *   - There is no NLB because there is nothing unstable to hide. The old design
 *     needed one because a Fargate task's awsvpc IP changed on every start; an
 *     instance's private IP survives stop/start and only moves on replacement.
 *     That deleted a $16/month resource and a whole hop.
 *   - It is a string, not a reference to the instance. An Fn::ImportValue on the
 *     instance's IP would freeze that value while this stack imports it, so
 *     replacing the instance — which a user-data change does by design — would
 *     fail with "Cannot update an export in use". Agreeing on an address instead
 *     of a resource keeps the two stacks independently deployable.
 *
 * The runtime's ENIs go in ISOLATED subnets with no default route at all — the
 * shared VPC has no NAT gateway. The runtime's own AWS API calls (Nova Sonic,
 * image pulls, logs) do not traverse these ENIs; they only need to reach
 * bridgeHost:8765, which is in-VPC. A public subnet would not work either, since
 * AgentCore ENIs never get a public IP.
 *
 * This is also what killed the old kvs_* tool group: Kinesis Video Streams has no
 * PrivateLink endpoint in us-east-1, so with no NAT those tools could never reach
 * the data endpoint from here. They have been removed rather than worked around —
 * describe_scene / compare_scenes hit Bedrock directly and were never affected.
 * See Go2NetworkStack's class doc for what was measured.
 */
export class Go2AgentCoreStack extends cdk.Stack {
  constructor(scope: Construct, id: string, props: Go2AgentCoreStackProps) {
    super(scope, id, props);

    applySolutionMetadata(
      this,
      "Unitree Go2 AWS robotics demo - the Nova Sonic voice agent hosted on " +
        "Amazon Bedrock AgentCore Runtime, in-VPC with the robot"
    );

    const bridgePort = props.bridgePort ?? 8765;
    const vpc = props.vpc;

    // --- Configuration ----------------------------------------------------
    // Resolved from context at SYNTH time (defaults in cdk.json), not from
    // CfnParameters. Both would work, but context needs no --parameters flags at
    // the call site, which is what let the deploy wrapper script go away; it also
    // keeps these values as real strings below rather than CloudFormation tokens,
    // so the container env and the KVS IAM ARN read literally in the template.
    //
    // Override per deploy with `--context key=value`.
    //
    // modelRegion is where Nova Sonic and the scene-description model resolve.
    // us-east-1 has both, so the demo stays single-region; the Bedrock IAM below
    // is region-wildcarded so pointing it elsewhere still works.
    const modelRegion: string = this.node.tryGetContext("modelRegion") ?? this.region;
    const voice: string = this.node.tryGetContext("novaSonicVoice") ?? "en-us.matthew";

    // --- Container image ---------------------------------------------------
    // Built from the repo root so `voice_agent/` lands under /app as an
    // importable package. ARM64 is an AgentCore Runtime requirement.
    const artifact = agentcore.AgentRuntimeArtifact.fromAsset("..", {
      file: "voice_agent/Dockerfile.agentcore",
      platform: cdk.aws_ecr_assets.Platform.LINUX_ARM64,
      exclude: [
        "deployment",
        // Glob, not a literal: host venvs are all named .venv*, and a stray
        // one left in the tree would otherwise be shipped into the image.
        ".venv*",
        "ros2_ws",
        "build",
        "install",
        "log",
        "**/__pycache__",
        "**/*.pyc",
        "*.ply",
        "*.png",
      ],
    });

    // --- Security group: egress to the bridge ------------------------------
    // The runtime's ENIs land in the private subnets. Go2Ec2Stack's instance SG
    // allows 8765 from the whole VPC CIDR, so no ingress pairing is needed here —
    // this group only governs the runtime's own egress.
    const runtimeSg = new ec2.SecurityGroup(this, "RuntimeSg", {
      vpc,
      description: "Go2 voice agent on AgentCore - foxglove bridge + AWS API egress",
      allowAllOutbound: true,
    });

    // --- The runtime -------------------------------------------------------
    // ProtocolType.HTTP is correct for WebSocket: the HTTP protocol contract
    // covers both /invocations and the /ws endpoint on port 8080, and the
    // container serves /ws + /ping via BedrockAgentCoreApp.
    const runtime = new agentcore.Runtime(this, "Go2VoiceAgent", {
      runtimeName: "go2_voice_agent",
      description: "Go2 quadruped voice agent (Nova Sonic, bidirectional WebSocket)",
      agentRuntimeArtifact: artifact,
      protocolConfiguration: agentcore.ProtocolType.HTTP,
      networkConfiguration: agentcore.RuntimeNetworkConfiguration.usingVpc(this, {
        vpc,
        vpcSubnets: { subnetType: ec2.SubnetType.PRIVATE_ISOLATED },
        securityGroups: [runtimeSg],
      }),
      environmentVariables: {
        // Robot link: the instance's fixed private IP, straight to the bridge.
        ROS_BRIDGE_HOST: props.bridgeHost,
        ROS_BRIDGE_PORT: String(bridgePort),
        // Compressed topic keeps scene grabs current (raw images go stale).
        CAMERA_TOPIC: "/camera/image_raw/compressed",
        // Nova Sonic + the scene model both resolve in this region.
        NOVA_SONIC_REGION: modelRegion,
        SCENE_REGION: modelRegion,
        NOVA_SONIC_VOICE: voice,
        AWS_DEFAULT_REGION: this.region,
        LOG_LEVEL: "INFO",
        // Solution attribution. voice_agent.config.boto_config() reads this and
        // passes it to botocore as user_agent_extra, so the agent's own Bedrock
        // (describe_scene / compare_scenes), Lambda and Secrets Manager calls
        // are attributable to this solution. Nova Sonic is the one exception —
        // see the note in voice_agent/agent.py's build_agent.
        USER_AGENT_STRING: solutionUserAgent(this),
        // voice_agent.config.guardrail_config() reads these and attaches the
        // guardrail to every converse() call on the vision path. Empty string
        // rather than omitted when unset: AgentRuntime environment values must
        // be strings, and config.py treats "" as "no guardrail configured".
        GUARDRAIL_ID: props.guardrailId ?? "",
        GUARDRAIL_VERSION: props.guardrailVersion ?? "DRAFT",
      },
      // Requires the ACCOUNT-WIDE X-Ray trace segment destination to be
      // CloudWatchLogs, otherwise the delivery resource fails with "X-Ray
      // Delivery Destination is supported with CloudWatch Logs as a Trace
      // Segment Destination". Set it once per account with:
      //   aws xray update-trace-segment-destination --destination CloudWatchLogs
      tracingEnabled: true,
    });

    // --- IAM: what the agent needs at run time -----------------------------
    // The L2 construct already grants ECR pull, CloudWatch Logs, X-Ray, and
    // workload identity. Everything below is this agent's own tool surface.

    // Nova Sonic (speech-to-speech) authorizes under bedrock:InvokeModel —
    // InvokeModelWithBidirectionalStream has no distinct IAM action. Same
    // statement covers the Claude model behind describe_scene / compare_scenes.
    // Region-wildcarded because these resolve in SCENE_REGION, not this stack's
    // region, and cross-region inference profiles fan out further still.
    runtime.role.addToPrincipalPolicy(
      new iam.PolicyStatement({
        sid: "BedrockModelInvoke",
        actions: [
          "bedrock:InvokeModel",
          "bedrock:InvokeModelWithResponseStream",
        ],
        resources: [
          `arn:aws:bedrock:*::foundation-model/amazon.nova-*`,
          `arn:aws:bedrock:*::foundation-model/anthropic.*`,
          `arn:aws:bedrock:*:${this.account}:inference-profile/*`,
        ],
      })
    );

    // Applying a guardrail authorizes separately from invoking the model: an
    // InvokeModel that carries guardrailIdentifier fails with AccessDenied
    // without this. Scoped to the one guardrail ARN, so no wildcard.
    if (props.guardrailArn) {
      runtime.role.addToPrincipalPolicy(
        new iam.PolicyStatement({
          sid: "ApplyGo2Guardrail",
          actions: ["bedrock:ApplyGuardrail"],
          resources: [props.guardrailArn],
        })
      );
    }

    // kvs_describe invokes Go2KVSStack's SceneDescriber. ListFunctions is how
    // kvs_tools._find_function resolves its generated name by prefix, and it cannot
    // be resource-scoped.
    //
    // Granted even though the call cannot currently leave these subnets: there is
    // no NAT and no Lambda interface endpoint in the shared VPC, so kvs_describe
    // works from a local `make voice` run and not from here. The grant means adding
    // that one endpoint is the only change needed — the alternative is a tool that
    // then fails on IAM instead, which is a worse thing to debug.
    //
    // No kinesisvideo grant: the agent never touches KVS directly. The robot
    // container uploads (Go2Ec2Stack holds PutMedia) and the Lambda reads.
    runtime.role.addToPrincipalPolicy(
      new iam.PolicyStatement({
        sid: "ListLambdasForKvsToolDiscovery",
        actions: ["lambda:ListFunctions"],
        resources: ["*"],
      })
    );
    runtime.role.addToPrincipalPolicy(
      new iam.PolicyStatement({
        sid: "InvokeGo2KVSLambdas",
        actions: ["lambda:InvokeFunction"],
        resources: [`arn:aws:lambda:*:${this.account}:function:Go2KVSStack-*`],
      })
    );

    // --- cdk-nag: what is deliberate here ----------------------------------
    // Split in two on purpose. The first group is the L2 construct's OWN grants —
    // logs, X-Ray, CloudWatch metrics, workload identity, the CDK asset repo —
    // which this stack does not write and cannot narrow without abandoning the L2.
    // The second is this agent's tool surface, which is ours to justify.
    NagSuppressions.addResourceSuppressions(
      runtime.role,
      [
        {
          id: "AwsSolutions-IAM5",
          reason:
            "Written by the agentcore.Runtime L2, not by this stack. The log " +
            "group name embeds the runtime id, which CloudFormation assigns at " +
            "deploy time, so the wildcards below are how the construct expresses " +
            "'this runtime's own logs and its own workload identity'.",
          appliesTo: [
            `Resource::arn:<AWS::Partition>:logs:${this.region}:${this.account}:log-group:/aws/bedrock-agentcore/runtimes/*`,
            `Resource::arn:<AWS::Partition>:logs:${this.region}:${this.account}:log-group:*`,
            `Resource::arn:<AWS::Partition>:logs:${this.region}:${this.account}:log-group:/aws/bedrock-agentcore/runtimes/*:log-stream:*`,
            `Resource::arn:<AWS::Partition>:bedrock-agentcore:${this.region}:${this.account}:workload-identity-directory/default/workload-identity/*`,
          ],
        },
        {
          id: "AwsSolutions-IAM5",
          reason:
            "Resource '*' here is three L2-written statements plus one of ours, " +
            "and none can be scoped: xray:PutTraceSegments / " +
            "PutTelemetryRecords / GetSamplingRules / GetSamplingTargets and " +
            "ecr:GetAuthorizationToken support no resource-level permissions; " +
            "cloudwatch:PutMetricData is instead condition-scoped to the " +
            "bedrock-agentcore namespace; and lambda:ListFunctions (ours, used by " +
            "kvs_tools._find_function to resolve the generated SceneDescriber " +
            "name by prefix) is a list operation with no resource form. The " +
            "matching lambda:InvokeFunction IS scoped.",
          appliesTo: ["Resource::*"],
        },
        {
          id: "AwsSolutions-IAM5",
          reason:
            "Model-family wildcards, not account wildcards. Nova Sonic and the " +
            "Claude behind describe_scene resolve in SCENE_REGION through a " +
            "cross-region inference profile, so region and exact model id both " +
            "move without a deploy; the vendor prefixes are the invariant. " +
            "InvokeModel is also the only action Nova Sonic's bidirectional " +
            "stream authorizes under — there is no narrower action to grant.",
          appliesTo: [
            "Resource::arn:aws:bedrock:*::foundation-model/amazon.nova-*",
            "Resource::arn:aws:bedrock:*::foundation-model/anthropic.*",
            `Resource::arn:aws:bedrock:*:${this.account}:inference-profile/*`,
          ],
        },
        {
          id: "AwsSolutions-IAM5",
          reason:
            "CloudFormation generates SceneDescriber's physical name, so the " +
            "prefix is what can be named without importing the ARN across the " +
            "stack boundary — which this design avoids on purpose (see the class " +
            "doc). Scoped to this account and to Go2KVSStack's functions.",
          appliesTo: [
            `Resource::arn:aws:lambda:*:${this.account}:function:Go2KVSStack-*`,
          ],
        },
      ],
      true
    );

    // --- Outputs ----------------------------------------------------------
    new cdk.CfnOutput(this, "AgentRuntimeArn", {
      value: runtime.agentRuntimeArn,
      description:
        "Export as AGENT_ARN, then: python -m voice_agent.agentcore_client",
    });

    new cdk.CfnOutput(this, "AgentRuntimeId", {
      value: runtime.agentRuntimeId,
      description: "AgentCore Runtime ID",
    });

    new cdk.CfnOutput(this, "ExecutionRoleArn", {
      value: runtime.role.roleArn,
      description: "Runtime execution role (Bedrock / Lambda permissions)",
    });

    new cdk.CfnOutput(this, "LogGroup", {
      value: `/aws/bedrock-agentcore/runtimes/${runtime.agentRuntimeId}-DEFAULT`,
      description: "CloudWatch log group for the agent",
    });
  }
}
