// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: MIT-0
import * as cdk from "aws-cdk-lib";
import * as bedrock from "aws-cdk-lib/aws-bedrock";
import { Construct } from "constructs";
import { applySolutionMetadata } from "./solution";

/**
 * The one Amazon Bedrock Guardrail every model invocation in this solution attaches.
 *
 * It has its own stack for two reasons, and the second is the reason it is not
 * folded into one of the others:
 *
 *   1. THREE consumers, no dependencies. The SceneDescriber Lambda
 *      (Go2KVSStack), the robot container (Go2Ec2Stack) and the hosted agent
 *      (Go2AgentCoreStack) all need the id. This stack imports nothing, so it
 *      deploys first and all three can consume it with no cycle.
 *   2. Naming honesty. This lived in Go2KVSStack at first, which meant a stack
 *      documented as "the robot's camera archive" silently owned the solution's
 *      AI safety control, and `make deploy-kvs` was the step that gated every
 *      model call in the system. Nobody would guess that from the name.
 *
 * ## What it is actually for
 *
 * This agent's tool surface actuates a physical robot, and its vision path reads a
 * camera pointed at a room. The prompt- and code-level defences — the
 * SCENE_SYSTEM_PROMPT wording, bedrock_utils.SYSTEM_GUARDRAIL,
 * notifications.quote_untrusted, the two-step gate in voice_agent/tools.py — all
 * assume the model cooperates with its instructions. This layer does not.
 *
 * `PROMPT_ATTACK` at `HIGH` on INPUT is the filter that carries the weight: it is
 * what screens text a visitor holds up in front of the camera, before the
 * description of it can come back looking like an instruction to the agent.
 * Input-only because that is the only direction the service supports for this
 * filter type.
 *
 * ## Reaching it from a local run
 *
 * Unset is a valid state everywhere — voice_agent/config.guardrail_config() and
 * bedrock_utils both degrade to unguarded calls rather than failing, so a
 * `make voice` run works in an account with no stacks. To attach the deployed
 * guardrail to a local run:
 *
 *   export GUARDRAIL_ID=$(aws cloudformation describe-stacks \
 *     --stack-name Go2GuardrailStack --query \
 *     "Stacks[0].Outputs[?OutputKey=='GuardrailId'].OutputValue" --output text)
 */
export class Go2GuardrailStack extends cdk.Stack {
  /** Guardrail id, for the consumers' GUARDRAIL_ID env var. */
  public readonly guardrailId: string;
  /** Guardrail version the consumers use. See the note on DRAFT below. */
  public readonly guardrailVersion: string;
  /** Guardrail ARN, for scoping `bedrock:ApplyGuardrail` in each consumer. */
  public readonly guardrailArn: string;

  constructor(scope: Construct, id: string, props?: cdk.StackProps) {
    super(scope, id, props);

    applySolutionMetadata(
      this,
      "Unitree Go2 AWS robotics demo - the Amazon Bedrock Guardrail applied to " +
        "every model invocation (prompt-attack screening and PII redaction)"
    );

    const guardrail = new bedrock.CfnGuardrail(this, "Go2Guardrail", {
      name: `${cdk.Stack.of(this).stackName}-go2-guardrail`,
      description:
        "Go2 robot demo - prompt-attack screening on input and PII redaction on " +
        "the vision path",
      blockedInputMessaging: "That request cannot be processed.",
      blockedOutputsMessaging: "That response was withheld.",
      contentPolicyConfig: {
        filtersConfig: [
          { type: "PROMPT_ATTACK", inputStrength: "HIGH", outputStrength: "NONE" },
          { type: "VIOLENCE", inputStrength: "MEDIUM", outputStrength: "MEDIUM" },
          { type: "MISCONDUCT", inputStrength: "MEDIUM", outputStrength: "MEDIUM" },
        ],
      },
      // ANONYMIZE rather than BLOCK: the robot describing "someone in a blue
      // jacket" is the feature, and blocking the whole response the moment a
      // model happens to guess at a name would make the greeter unusable. This
      // redacts the identifier and keeps the description.
      sensitiveInformationPolicyConfig: {
        piiEntitiesConfig: [
          { type: "NAME", action: "ANONYMIZE" },
          { type: "EMAIL", action: "ANONYMIZE" },
          { type: "PHONE", action: "ANONYMIZE" },
        ],
      },
    });

    // A guardrail has to be versioned before anything can reference it by a
    // numbered version. DRAFT is deliberately what the consumers use: it tracks
    // edits to the filters above without redeploying all three of them, which is
    // the right trade for a sample whose filters a reader is expected to tune.
    // PIN THIS VERSION'S NUMBER FOR PRODUCTION — see RESPONSIBLE-AI.md.
    const version = new bedrock.CfnGuardrailVersion(this, "Go2GuardrailVersion", {
      guardrailIdentifier: guardrail.attrGuardrailId,
      description: "Deployed version of the Go2 guardrail",
    });
    version.node.addDependency(guardrail);

    this.guardrailId = guardrail.attrGuardrailId;
    this.guardrailArn = guardrail.attrGuardrailArn;
    this.guardrailVersion = "DRAFT";

    new cdk.CfnOutput(this, "GuardrailId", {
      value: this.guardrailId,
      description: "Bedrock Guardrail id - export as GUARDRAIL_ID for a local run",
    });

    new cdk.CfnOutput(this, "GuardrailArn", {
      value: this.guardrailArn,
      description: "Bedrock Guardrail ARN (grant bedrock:ApplyGuardrail on this)",
    });

    new cdk.CfnOutput(this, "GuardrailVersion", {
      value: this.guardrailVersion,
      description: "Version the consumers attach. DRAFT tracks edits; pin for prod",
    });
  }
}
