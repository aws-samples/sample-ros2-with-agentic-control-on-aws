// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: MIT-0
import * as cdk from "aws-cdk-lib";
import * as kinesisvideo from "aws-cdk-lib/aws-kinesisvideo";
import * as lambda from "aws-cdk-lib/aws-lambda";
import * as iam from "aws-cdk-lib/aws-iam";
import * as logs from "aws-cdk-lib/aws-logs";
import { NagSuppressions } from "cdk-nag";
import { Construct } from "constructs";
import * as path from "path";
import { applySolutionMetadata, solutionUserAgent } from "./solution";

export interface Go2KVSStackProps extends cdk.StackProps {
  /**
   * Robot identifier — names the KVS stream (`<deviceId>-camera`). Resolved from
   * context in bin/app.ts and shared with Go2Ec2Stack, whose container writes to
   * the same name. Required rather than defaulted so the default lives in exactly
   * one place.
   */
  readonly deviceId: string;
  /**
   * The Bedrock Guardrail from Go2GuardrailStack, attached to every invoke_model
   * call the SceneDescriber makes (bedrock_utils reads these from the env).
   *
   * Optional so this stack still deploys standalone against an account where
   * Go2GuardrailStack has not been created — bedrock_utils degrades to unguarded
   * calls rather than failing. bin/app.ts always supplies them.
   */
  readonly guardrailId?: string;
  readonly guardrailVersion?: string;
  readonly guardrailArn?: string;
}

/**
 * The robot's camera archive — one Kinesis Video Stream — plus the one Lambda that
 * reads a frame back out of it.
 *
 * `kvs_producer_node` in the container (Go2Ec2Stack sets KVS_IMAGE_TOPIC=
 * /annotated_image) encodes the COCO overlay feed to H.264 and PutMedia's it here.
 * Mostly the consumers are humans — the HLS viewer in the README.
 *
 * SceneDescriber is the exception. The voice agent invokes it directly, it pulls
 * frames with GetClip + ffmpeg, asks Bedrock about them, and returns text for Nova
 * Sonic to speak. Two modes:
 *
 *   - `kvs_describe` — newest frame. Duplicates what the agent's own describe_scene
 *     does off the foxglove bridge, deliberately: going through the stream is the
 *     point when demoing the cloud path.
 *   - `kvs_compare_scenes` — a frame from N seconds ago AND the newest one, both in
 *     front of the model, reporting what changed. This one is not a duplicate. The
 *     agent's local compare_scenes can only reach a moment it snapshotted during the
 *     session; this reads the 24 h archive, so it answers "what changed since this
 *     morning?" — which is the reason to retain recorded video at all.
 *
 * This stack used to hold considerably more: three container-image Lambdas
 * (describe / recall / monitor) behind IoT topic rules, a DynamoDB baseline table,
 * a 1-minute EventBridge schedule, and an MQTT response leg for a ROS2 node to
 * speak. All of that is gone except the describer, because none of it worked:
 *
 *   - SceneMonitor sat at reserved concurrency 0 while its schedule kept firing:
 *     1,440 events/day received and 1,440 dropped, for months.
 *   - SceneRecaller never produced a single log stream.
 *   - The MQTT→ROS2 response leg (edge/ros2_iot_bridge.py) was orphaned at both
 *     ends: no publisher for /kvs_trigger, and its /tts subscriber (the
 *     speech_processor package) was deleted once Nova Sonic's speech-to-speech made
 *     a TTS node redundant. So the describer now just returns its answer.
 *
 * Caveat worth knowing before relying on kvs_describe: the HOSTED agent cannot
 * invoke it end-to-end. Its ENIs sit in isolated subnets with no NAT, and while
 * Lambda has a PrivateLink endpoint, this stack does not add one — see
 * Go2AgentCoreStack. It works from a local `make voice` run.
 */
export class Go2KVSStack extends cdk.Stack {
  constructor(scope: Construct, id: string, props: Go2KVSStackProps) {
    super(scope, id, props);

    applySolutionMetadata(
      this,
      "Unitree Go2 AWS robotics demo - robot camera archive (Kinesis Video " +
        "Streams) and the scene-description Lambda that reads frames back out of it"
    );

    const deviceId = props.deviceId;

    // 24-hour retention: enough to scrub back through a demo session. The name is
    // this resource's physical id, so changing deviceId REPLACES the stream.
    const stream = new kinesisvideo.CfnStream(this, "RobotCameraStream", {
      name: `${deviceId}-camera`,
      dataRetentionInHours: 24,
      mediaType: "video/h264",
    });

    // Declared explicitly rather than via the function's `logRetention` prop. That
    // prop is deprecated, and it works by dropping a whole extra singleton Lambda
    // (plus role and policy) into the stack that calls PutRetentionPolicy at deploy
    // time — four resources and a custom-resource round trip to set one number.
    // A real LogGroup sets it directly. Matches AutoStopLogGroup in Go2Ec2Stack.
    const describerLogs = new logs.LogGroup(this, "SceneDescriberLogGroup", {
      retention: logs.RetentionDays.ONE_WEEK,
      removalPolicy: cdk.RemovalPolicy.DESTROY,
    });

    // Its own role rather than the one DockerImageFunction would build, and the
    // difference is one managed policy: CDK's default role attaches
    // AWSLambdaBasicExecutionRole, which grants logs:CreateLogGroup /
    // CreateLogStream / PutLogEvents on Resource "*" — every log group in the
    // account (AwsSolutions-IAM4). Since the log group is declared above, the
    // grant can name it, and CreateLogGroup is not needed at all.
    const describerRole = new iam.Role(this, "SceneDescriberRole", {
      assumedBy: new iam.ServicePrincipal("lambda.amazonaws.com"),
      description: "SceneDescriber - own log group, this robot's KVS stream, Bedrock",
    });
    describerLogs.grantWrite(describerRole);

    // Scene Describer — "what does the cloud see?". Container image because it
    // needs a real ffmpeg binary to cut a JPEG out of the GetClip MKV.
    const sceneDescriber = new lambda.DockerImageFunction(this, "SceneDescriber", {
      role: describerRole,
      code: lambda.DockerImageCode.fromImageAsset(
        path.join(__dirname, "../lambdas"),
        {
          file: "scene-describer/Dockerfile",
          platform: cdk.aws_ecr_assets.Platform.LINUX_AMD64,
        }
      ),
      architecture: lambda.Architecture.X86_64,
      // Compare mode is the expensive path: two GetClips, two ffmpeg extractions,
      // and a two-image Bedrock call. 60s was enough for describe alone and is not
      // reliably enough for that, especially on a cold start pulling a 330 MB image.
      timeout: cdk.Duration.seconds(120),
      memorySize: 1024,
      environment: {
        STREAM_NAME: `${deviceId}-camera`,
        BEDROCK_MODEL_ID: "us.anthropic.claude-sonnet-4-6",
        // bedrock_utils attaches these to every invoke_model call. Empty string
        // rather than omitted when unset: it treats "" as "no guardrail
        // configured" and degrades to unguarded calls, which is what keeps the
        // module usable outside this stack.
        GUARDRAIL_ID: props.guardrailId ?? "",
        GUARDRAIL_VERSION: props.guardrailVersion ?? "DRAFT",
        // Solution attribution for the KVS and Bedrock calls this function
        // makes. kvs_utils / bedrock_utils read it and pass it to botocore as
        // user_agent_extra; unset would silently drop this function out of the
        // solution's API-usage reporting, hence the mapping rather than a
        // literal repeated per runtime.
        USER_AGENT_STRING: solutionUserAgent(this),
      },
      logGroup: describerLogs,
    });

    // GetClip is the read path; the rest are what the SDK touches getting there.
    // All four scoped to this robot's stream by ARN — including GetDataEndpoint,
    // which the SDK addresses by NAME but IAM authorizes against the stream
    // resource like the others. The stream is created above, so its real ARN is
    // available and no wildcard is needed at all.
    sceneDescriber.addToRolePolicy(
      new iam.PolicyStatement({
        sid: "KinesisVideoReader",
        actions: [
          "kinesisvideo:GetDataEndpoint",
          "kinesisvideo:DescribeStream",
          "kinesisvideo:GetClip",
          "kinesisvideo:GetImages",
        ],
        resources: [stream.attrArn],
      })
    );
    sceneDescriber.addToRolePolicy(
      new iam.PolicyStatement({
        sid: "BedrockDescribeFrame",
        actions: ["bedrock:InvokeModel"],
        resources: [
          `arn:aws:bedrock:*::foundation-model/anthropic.*`,
          `arn:aws:bedrock:*:${this.account}:inference-profile/*`,
        ],
      })
    );

    // Applying a guardrail is a separate authorization from invoking the model:
    // without ApplyGuardrail on the guardrail's own ARN, an invoke_model that
    // carries guardrailIdentifier fails with AccessDenied. Scoped to the one
    // guardrail ARN, so no wildcard and no nag suppression needed.
    if (props.guardrailArn) {
      sceneDescriber.addToRolePolicy(
        new iam.PolicyStatement({
          sid: "ApplyGo2Guardrail",
          actions: ["bedrock:ApplyGuardrail"],
          resources: [props.guardrailArn],
        })
      );
    }

    // --- cdk-nag: what is deliberate here ----------------------------------
    NagSuppressions.addResourceSuppressions(
      describerRole,
      [
        {
          id: "AwsSolutions-IAM5",
          reason:
            "Model-family wildcard, not an account wildcard. BEDROCK_MODEL_ID is " +
            "a Claude model reached through a cross-region inference profile, so " +
            "the exact model id and the region both move without a deploy; the " +
            "vendor prefix (anthropic.) is the invariant worth pinning.",
          appliesTo: [
            "Resource::arn:aws:bedrock:*::foundation-model/anthropic.*",
            `Resource::arn:aws:bedrock:*:${this.account}:inference-profile/*`,
          ],
        },
      ],
      true
    );

    // The one PrototypeSecurityNagPack error. This function is outside the VPC on
    // purpose, and it is not a preference: its whole job is GetClip against a KVS
    // DATA endpoint, and Kinesis Video Streams has no PrivateLink endpoint
    // (kinesisvideo, -media and -signaling are all InvalidServiceName in this
    // region). In a subnet it would need a NAT gateway (~$33/month) to do the one
    // thing it exists to do, and the shared VPC deliberately has none — see
    // Go2NetworkStack. Nothing reaches it from the network either way: it is
    // invoke-only, with no URL and no public trigger.
    NagSuppressions.addResourceSuppressions(sceneDescriber, [
      {
        id: "Prototype Security Nag Pack-LambdaInsideVPC",
        reason:
          "Reads the KVS data endpoint, which has no PrivateLink endpoint in " +
          "this region. In-VPC it would require a NAT gateway the shared VPC " +
          "does not have. Invoke-only: no function URL, no public trigger.",
      },
    ]);

    new cdk.CfnOutput(this, "KvsStreamName", {
      value: `${deviceId}-camera`,
      description: "KVS stream name for the robot camera",
    });

    new cdk.CfnOutput(this, "SceneDescriberFunctionName", {
      value: sceneDescriber.functionName,
      description: "Lambda behind the voice agent's kvs_describe tool",
    });
  }
}
