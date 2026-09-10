#!/usr/bin/env node
// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: MIT-0
import "source-map-support/register";
import * as cdk from "aws-cdk-lib";
import { Go2GuardrailStack } from "../lib/go2-guardrail-stack";
import { Go2KVSStack } from "../lib/go2-kvs-stack";
import { Go2NetworkStack, ROBOT_PRIVATE_IP } from "../lib/go2-network-stack";
import { Go2Ec2Stack } from "../lib/go2-ec2-stack";
import { Go2AgentCoreStack } from "../lib/go2-agentcore-stack";
import { AwsSolutionsChecks } from 'cdk-nag';
import { PrototypeSecurityNagPack } from "./prototype-security";

const app = new cdk.App();

// Applied to every taggable resource in all stacks (propagates down the
// construct tree). Marks resources as exempt from automated cleanup sweeps.
cdk.Tags.of(app).add("auto-delete", "no");

const env = {
  account: process.env.CDK_DEFAULT_ACCOUNT,
  region: process.env.CDK_DEFAULT_REGION || "us-east-1",
};

// One robot identifier, resolved once here and passed down as a required prop so
// the copies cannot drift. It names the KVS stream, `<deviceId>-camera`:
// Go2KVSStack creates it and the robot container in Go2Ec2Stack writes to it.
//
// Disagreement is silent — the container uploads to a stream nobody is watching.
// (Go2AgentCoreStack no longer takes it: the kvs_* voice tools that read the
// stream are gone, so the agent has nothing to address by name.)
// Previously this was a separate CfnParameter in each stack, i.e. one place per
// stack to keep in sync by hand. Override everywhere at once:
//   npx cdk deploy … --context deviceId=go2-robot-02
//
// Changing it REPLACES the KVS stream (the name is its physical id), so treat it
// as a new-robot operation, not a rename.
const deviceId: string = app.node.tryGetContext("deviceId") ?? "go2-robot-01";

// --- The AI safety control every model call attaches ----------------------
// One Bedrock Guardrail, in its own stack because all THREE other stacks need its
// id and it imports nothing itself — so it deploys first and none of them create a
// cycle. It started out inside Go2KVSStack, which worked but meant a stack
// documented as "the robot's camera archive" silently owned the guardrail and
// `make deploy-kvs` was the step gating every model call in the system.
const guardrail = new Go2GuardrailStack(app, "Go2GuardrailStack", { env });

const guardrailProps = {
  guardrailId: guardrail.guardrailId,
  guardrailVersion: guardrail.guardrailVersion,
  guardrailArn: guardrail.guardrailArn,
};

// Serverless and now nearly empty: the KVS stream the robot uploads to plus the one
// Lambda that reads frames back out. The scene-analysis Lambdas and IoT rules it
// used to hold are gone — see the stack.
const kvs = new Go2KVSStack(app, "Go2KVSStack", {
  env,
  deviceId,
  ...guardrailProps,
});

// --- The VPC everything else shares ---------------------------------------
// There used to be two, one per compute stack, both on 10.0.0.0/16 — so they
// could never have been peered, and the hosted agent could only ever reach
// whichever one it had been deployed into. One VPC, built once, passed down.
const network = new Go2NetworkStack(app, "Go2NetworkStack", { env });

// --- The robot ------------------------------------------------------------
// One EC2 instance in the shared VPC's public subnet, at a fixed private IP.
// Replaced an ECS Fargate stack whose NAT + internal NLB + SSM jumpbox cost
// ~$57/month and existed only to work around a Fargate task having no host and
// no stable address. See lib/go2-ec2-stack.ts.
const robot = new Go2Ec2Stack(app, "Go2Ec2Stack", {
  env,
  deviceId,
  vpc: network.vpc,
  robotSubnet: network.robotSubnet,
  robotPrivateIp: ROBOT_PRIVATE_IP,
});

// --- The hosted voice agent -----------------------------------------------
// Joins the same VPC (isolated subnets, no NAT — its own AWS calls don't use these
// ENIs) and reaches the robot's foxglove bridge directly at ROBOT_PRIVATE_IP:8765 —
// no load balancer, no SSM tunnel on its path.
//
// Note what is passed: the VPC as a construct, but the bridge host as a plain
// STRING. A reference to the instance's IP would synthesize an Fn::ImportValue,
// which would then block any update that changed it — and replacing the instance
// is a normal operation here (a user-data edit does it). Agreeing on an address
// rather than on a resource keeps these two independently deployable.
//
// The explicit dependency is for ordering only: the bridge should exist before
// the agent that dials it. Nothing is imported across the boundary.
const agent = new Go2AgentCoreStack(app, "Go2AgentCoreStack", {
  env,
  vpc: network.vpc,
  bridgeHost: ROBOT_PRIVATE_IP,
  ...guardrailProps,
});
agent.addStackDependency(robot);

cdk.Aspects.of(app).add(new AwsSolutionsChecks({ verbose: true }));
cdk.Aspects.of(app).add(new PrototypeSecurityNagPack({ verbose: true, reports: true }));
