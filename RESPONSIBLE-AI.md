# Responsible AI

This sample is unusual in two ways that matter more than the model choice:

1. **A language model's output drives a physical actuator.** Nova Sonic decides
   which tools to call, and those tools move ~15 kg of robot at up to 0.5 m/s.
   There is no human in the loop between the model's decision and the robot's legs.
2. **The model reads the room.** A camera pointed at a space with people in it
   feeds a vision model whose text comes back into the agent's context. That makes
   anything visible in the room — including printed text — an input.

Everything below follows from those two facts. Read this alongside the
non-production disclaimer at the top of [README.md](README.md).

---

## Intended use

- Demonstrating a voice-driven agent controlling a ROS 2 robot with AWS services.
- Learning how Amazon Nova Sonic, Amazon Bedrock vision models, Amazon Bedrock
  AgentCore Runtime, and a ROS 2 stack fit together.
- Running in a **controlled space** — a lab, a demo area, a cleared stage — with an
  operator present, aware the robot is running, and able to reach it.

## Out of scope

Do not use this, as written, for:

- **Anything safety-critical.** There is no functional-safety argument here, no
  certified stop, and no obstacle avoidance on the movement path. `FIND_APPROACH`
  is off by default for exactly this reason: the Go2's LiDAR cannot reliably
  support walking toward something.
- **Unattended or autonomous operation.** Greeter mode runs on its own, but it is
  meant to run while someone is watching it.
- **Surveillance, or identifying people.** The prompts and the guardrail actively
  work against identification (see below), and repurposing this to identify people
  means removing controls that are there on purpose.
- **Public or uncontrolled spaces.** Bystanders cannot consent to a robot that
  photographs them and can flip.
- **Any deployment where the robot can reach a person who is not expecting it.**

## How model output reaches the actuators

```
microphone ─▶ Nova Sonic ─▶ tool call ─▶ Go2ROS2Client ─▶ foxglove_bridge ─▶ robot
                  ▲
camera ─▶ Bedrock vision ─┘   (descriptions re-enter the model's context)
```

Three controls sit on that path, in code, not in prose:

| Control | Where | What it does |
| --- | --- | --- |
| Velocity clamp | `config.MAX_LINEAR_VELOCITY` (0.5 m/s), `MAX_ANGULAR_VELOCITY` (0.8 rad/s), applied in the client | No tool argument can command a faster robot |
| Motion allowlist | `perform_special_motion` in the client | Only named sport motions are issuable; the model cannot compose arbitrary commands |
| Two-step gate on ballistic motion | `voice_agent/tools.py` | Flips, jumps and pounces need `request_dangerous_action` and then `confirm_dangerous_action` in a **later turn**, naming the same action within 60 s, with the user's spoken confirmation in between. `perform_action` refuses them outright |
| Direction validation | `move` in `voice_agent/tools.py` | An unrecognised direction is an error, never a default. Nothing that actuates defaults to moving |

The two-step gate exists because prose in a system prompt is advice to a model, not
a control. A single injected instruction bypasses advice; it cannot bypass a gate
that requires two turns and a human utterance between them.

## Prompt injection, and why the camera is the threat

The realistic attack on this system needs no account, no network access, and no
credential — only line of sight. A visitor holds up a printed sign; the vision
model reads it; the description comes back into the agent's context, where the
system prompt has told the agent that robot events are to be acted on.

Four layers address this, deliberately overlapping:

1. **The vision prompts refuse to relay image text.** `SCENE_SYSTEM_PROMPT`,
   `GREETER_VISITOR_PROMPT` (`voice_agent/config.py`) and `SYSTEM_GUARDRAIL`
   (`deployment/lambdas/shared/python/bedrock_utils.py`) all instruct the model to
   note that text is present, never to transcribe or follow it.
2. **Model output is never interpolated into an instruction.** Robot events carry a
   fixed instruction first and quote the model's text after it, labelled untrusted,
   via `notifications.quote_untrusted()`, which strips bracket punctuation and
   control vocabulary and bounds the length.
3. **The `[robot event]` marker cannot be forged by a client.** It is a trust grant
   that only this process may mint; `notifications.is_forged()` rejects any client
   frame or typed line containing it, at both the WebSocket and stdin boundaries.
4. **An Amazon Bedrock Guardrail screens every vision call.** `PROMPT_ATTACK` at
   `HIGH` on input is the filter that matters, and it is the only layer that still
   applies if a model ignores its system prompt.

**Known gap:** the Nova Sonic bidirectional stream itself carries **no** guardrail.
`BidiNovaSonicModel` exposes no path to one, and passing an unknown config key
would silently do nothing — an acknowledged gap is better than a fake control. See
the note in `build_agent` (`voice_agent/agent.py`). The guarded layer is the one
that matters most here, since untrusted content enters through the camera rather
than the microphone.

## Guardrail configuration

Created in `deployment/lib/go2-kvs-stack.ts`, passed to the EC2 container and the
AgentCore runtime as `GUARDRAIL_ID` / `GUARDRAIL_VERSION`, and attached by
`config.guardrail_config()` and `bedrock_utils`.

| Policy | Setting | Why |
| --- | --- | --- |
| `PROMPT_ATTACK` | input `HIGH`, output `NONE` | The camera-injection path. Input-only is the only direction the service supports for this filter |
| `VIOLENCE` | input/output `MEDIUM` | Ordinary content hygiene on a spoken channel |
| `MISCONDUCT` | input/output `MEDIUM` | Same |
| `NAME`, `EMAIL`, `PHONE` | `ANONYMIZE` | Redact identifiers but keep the description. `BLOCK` would make the greeter unusable the moment a model guessed at a name |

The version is `DRAFT` on purpose: a reader is expected to tune these filters, and
`DRAFT` tracks edits without redeploying three consumers. **Pin a numbered version
before production.** Unset is also a valid state — the code degrades to unguarded
calls rather than failing — which keeps a local run working in an account with no
stacks, and which you should not rely on.

## Known failure modes

| Failure | What it looks like | Mitigation in this repo |
| --- | --- | --- |
| The model invents a result | "I found your backpack to my left" when nothing was found | The system prompt forbids it, tools return explicit `status`/`note` the agent must report, and find events say "do not invent a location" |
| A misdetection triggers a social gesture | The robot waves at a coat rack | `GREETER_MIN_SCORE` (0.6, above the detector's own floor) plus a multi-frame debounce |
| The vision model misdescribes a person | Wrong clothing colour, or a guess at identity | Prompts forbid identification; the guardrail anonymises names |
| A Bedrock call fails and looks like an empty room | "I don't see anyone" when the real cause is `AccessDenied` | Failures log a full traceback (`logger.exception`) and return a fixed, non-diagnostic message; the log is where the difference lives |
| Speech recognition mishears a command | The robot does something adjacent to what was asked | Ballistic motions need explicit two-turn confirmation; everything else is velocity-clamped and time-bounded (≤10 s per move) |
| The robot walks into something | It has no obstacle avoidance on the move path | `FIND_APPROACH` off by default; movement is short and operator-supervised |
| Unbounded spend on a long session | A growing Bedrock bill | Per-call token caps only. Documented, not solved — see [Cost and rate limits](README.md#cost-and-rate-limits) |

## Human oversight this assumes

The design assumes, and does not enforce:

- An operator is **present and watching** whenever the robot is powered on.
- The space is **cleared** — at least 2 m in every direction before any flip or
  jump, on a flat non-slip floor.
- Someone can physically reach the robot, or its remote, to stop it. `make stop-ec2`
  and `stop_moving` are software; the physical remote is the real stop.
- People who may appear in front of the camera have been told the system is running.

## Data handling

Camera frames, spoken audio, and the model's descriptions of people are all
personal data when people are present. What is captured, where it goes, how long it
stays, and what remains your responsibility are set out in
[Imagery, people and privacy](README.md#imagery-people-and-privacy).

Transcribed speech is **not** logged. Tool call sites log metadata only — see the
note at the top of `voice_agent/tools.py`, and keep it that way.

## Reporting a concern

This is sample code with no SLA. For a security issue in this repository follow
[CONTRIBUTING.md](CONTRIBUTING.md); for an AWS service issue use the AWS Security
team's [vulnerability reporting](https://aws.amazon.com/security/vulnerability-reporting/)
process.
