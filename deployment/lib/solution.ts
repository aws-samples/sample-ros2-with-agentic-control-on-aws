// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: MIT-0
import * as cdk from "aws-cdk-lib";

/**
 * AWS Solution identity — the one place the solution ID and version are written.
 *
 * Two separate reporting paths depend on the values below, and both are string
 * matches, so nothing here is cosmetic:
 *
 *   1. CloudFormation metrics. Every template in this app carries a description
 *      of the form "(SO0367) - <what it is>. Version v<x.y.z>", which is how a
 *      deploy of this solution is attributed. Set via applySolutionMetadata().
 *   2. Service API usage. Every AWS SDK call made by code this app deploys
 *      appends "AWSSOLUTION/SO0367/v<x.y.z>" to the User-Agent header. The
 *      string reaches that code as the USER_AGENT_STRING environment variable,
 *      read from the CustomUserAgent mapping — see solutionUserAgent().
 *
 * On release, bump SOLUTION_VERSION here and nowhere else — with the one
 * exception noted below.
 *
 * The exception: the Python side (voice_agent, go2_robot_sdk, the Lambda
 * helpers) keeps a fallback copy of this string for runs that are NOT deployed
 * by this app — `make voice` on a laptop, docker-compose on a Mac, a ROS launch
 * against the robot over the LAN. Those processes have no CloudFormation
 * mapping to read, so each Python entry point defines the same constants and
 * prefers USER_AGENT_STRING when it is set. `grep -rn AWSSOLUTION` finds all of
 * them; they all point back here.
 */
export const SOLUTION_ID = "SO0367";

/** Bumped per release. Keep in step with deployment/package.json's version. */
export const SOLUTION_VERSION = "v1.0.0";

/** The string that goes into the User-Agent header of every SDK call. */
export const CUSTOM_USER_AGENT = `AWSSOLUTION/${SOLUTION_ID}/${SOLUTION_VERSION}`;

/** Logical id of the per-stack mapping holding CUSTOM_USER_AGENT. */
const MAPPING_ID = "Solution";

/**
 * Stamp a stack's template description with the solution ID and version.
 *
 * `description` is the human half — say what the stack IS, without a trailing
 * full stop or a version; both are added here. Call it first in the
 * constructor, once per stack, so the format cannot drift between templates.
 */
export function applySolutionMetadata(stack: cdk.Stack, description: string): void {
  stack.templateOptions.description = `(${SOLUTION_ID}) - ${description}. Version ${SOLUTION_VERSION}`;
}

/**
 * The custom user-agent string as a CloudFormation Fn::FindInMap reference.
 *
 * Returns a token, not a literal, on purpose: the value lands in exactly one
 * place per template (the Solution mapping) and every environment variable that
 * carries it points at that entry. A release then changes one line of one
 * template rather than one line per runtime.
 *
 * The mapping is created on first use and reused after, so stacks that deploy
 * no code (Go2NetworkStack) never render an unused mapping.
 */
export function solutionUserAgent(stack: cdk.Stack): string {
  const existing = stack.node.tryFindChild(MAPPING_ID) as cdk.CfnMapping | undefined;
  const mapping =
    existing ??
    new cdk.CfnMapping(stack, MAPPING_ID, {
      mapping: { Metadata: { CustomUserAgent: CUSTOM_USER_AGENT } },
    });
  return mapping.findInMap("Metadata", "CustomUserAgent");
}
