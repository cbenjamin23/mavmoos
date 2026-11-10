# Admission boundary, invalidation, handoff, and recovery

October 28, 2026. Proposed resolutions for the remaining six focused questions. These depend on the [coherent-output](helm-output.md) and [origin-freshness](freshness.md) recommendations. They are not adopted deployment policy, selected library behavior, or runtime guarantees.

## 1. Trusted ingress: Q-SH-01

**Recommendation:** start with a declared, controlled deployment boundary: one provisioned vehicle context, an allowlisted logical producer or explicitly selected standby, and reviewed communication paths. Require a configured association between producer incarnation, ingress, bridge epoch, vehicle session and permitted scopes before admitting effectful envelopes. Observation-only telemetry may remain available with weaker provenance clearly reported.

This is an initial analysis/deployment profile, not a limitation on the project's long-term multi-vehicle scope. If a relay or multi-community path is included, enumerate it in the boundary. A relay must preserve the canonical body, original decision/request identity, window, authority binding and validity. It can add separate forwarding diagnostics. Changing the outer MOOS source/community/time must not change the identity of the original decision.

The selected Core MOOS path ordinarily fills in a client source name, but also contains source-preservation/fake-source options. MOOSDB records source/auxiliary/community metadata. These fields are useful attribution claims; this source review does not establish authenticated producer identity or deployment access control. A window ID read from MOOSDB is not permission. [E5](evidence.md#e5-core-moos-messages-mail-and-retention).

For the proposed effectful profile, admission needs all of:

1. A provisioned issuer/path association whose trust assumptions are stated and enforced for that deployment.
2. An active session exchange that binds this producer incarnation to the current receiving bridge and vehicle context. Retained publications alone cannot establish a new session.
3. Scope authorization independent of claimed IDs, current deadlines and revocation state.
4. Original-age evidence from the permitted producer, not merely the last intermediary.

A controlled host/network can be an explicitly assumed trust boundary; it must not be advertised as an adversarial identity guarantee. For less trusted or shared paths, select and assess an authenticated/integrity-protected mechanism before claiming that stronger guarantee. This review does not select credentials, cryptography, or a MOOS transport replacement.

If the original issuer/session cannot be established through a relay, classify the observation's provenance accordingly and reject the strict effectful route. Do not silently downgrade it to arrival grouping while continuing to advertise the stronger profile.

**Still open:** actual binding/exchange/enforcement mechanism, relay audit, authorization configuration, revocation lifecycle, and deployment assumptions. Q-AUTH-01's complete scope/conflict/bypass inventory is also required before implementing the profile.

## 2. Gaps and invalidation: Q-NAV-02

**Recommendation:** distinguish missing new decisions from an explicit invalid outcome. A valid producer record declaring incomplete/empty/halted/all-stop or a state that removes navigation eligibility invalidates that producer's current intent immediately at the bridge's ordered admission boundary. Numeric fields in such a record are not a zero-speed/zero-altitude action.

The inspected helm deliberately tolerates an initial incomplete/empty or no-goal decision before scalar zeroing, and suppresses repeated identical all-stop reasons. Native scalar behavior therefore includes a transition grace that must not become an undocumented bridge policy. The normal output path includes `m_helm_prefix`; the all-stop scalar loop does not. [E1](evidence.md#e1-helm-cycle-and-output-order), [E7](evidence.md#e7-all-stop-and-standby).

| Event | Proposed effect on current intent | Additional action |
| --- | --- | --- |
| New complete valid decision from current producer | Replace the whole snapshot if fresh and permitted | Continue only within current grant/profile |
| Same decision retransmitted | No renewal | Retain original deadline |
| No new decision, old one still eligible | Only the profile's explicitly bounded continuation, if adopted | Never extend origin/window/grant limits |
| Explicit invalid outcome from current permitted producer | Remove that producer's navigation eligibility; do not wait for old expiry | Apply named profile response under its own permission |
| Intent/window/grant expires | Routine navigation under it becomes ineligible | Deadline-driven release/failure policy; no automatic replay |
| Malformed/unauthorized/old-epoch record | Reject as a candidate effect | Cannot itself revoke another producer's valid grant; apply configured stream-integrity/loss policy |

For the first strict profile, recommend **no continuation after explicit invalidation** and no new generic grace duration. If a profile needs a bounded gap continuation, explain which silent-gap cases it covers, the maximum remaining original age, and why it is acceptable for that vehicle/control level. Continuation is a permission to use an already admitted object briefly, not proof of a new decision.

Invalidation cannot instantaneously undo an in-flight target or require the FC to forget it. A separately permitted release/failure action may be required by the chosen profile. Do not run such an action under the revoked producer's grant. This preserves the distinction already proposed in the [authority draft](../../contracts/2026-10-28-draft-01/authority.md).

**Still open:** a vehicle-specific gap/failure policy, detection/response lateness bounds, and qualified FC response. No universal Loiter, zero vector, disarm, or target-hold action is selected.

## 3. Standby and operator arbitration: Q-AUTH-05

**Recommendation:** the bridge owns its scoped authority generations. Native helm `DRIVE`, `DRIVE+`, `PARK`, `STANDBY` and `DISABLED` are producer evidence, not grant issuance. A standby heartbeat or a newly arrived decision must not select the active owner by arrival order.

The [MIT helm chapter](https://oceanai.mit.edu/ivpman/pdfs/chap_helm_as_moos.pdf), extracted lines 585–623, explicitly describes a temporarily hung primary publishing after standby takeover, followed by the old primary's all-stop. The documented controller assumption tolerates that transient disturbance. MAVMOOS need not adopt that assumption for a strict navigation profile. The inspected source implements source-name-based helm coordination and delayed-primary refresh/all-stop handling. [E7](evidence.md#e7-all-stop-and-standby), [M1](evidence.md#m1-mit-manuals).

Proposed bridge transfer order:

1. Select the transfer trigger under an explicit priority policy. Record loss, operator request or standby request as evidence, not successful acquisition.
2. Revoke the old producer's routine permission and invalidate its controllable queued navigation. Serialize this change with admission and final egress. Record any residual old work.
3. Create a new authority epoch/grant and bind the selected producer/session/ingress, vehicle session and profile. Issue fresh origin windows for that binding.
4. Require a new coherent decision begun under that binding and fresh prerequisite observations. A recently completed old-epoch decision cannot simply be relabeled.
5. Perform only the profile's permitted acquisition/preparation sequence. Enter ACTIVE only when its fresh guards are satisfied. The same acquisition grant may become active if its defined phase permissions allow it; a new decision does not grant that transition itself.
6. Reject subsequent old-primary/old-epoch records as current navigation, including its final scalar all-stop. They remain historical evidence. An old source cannot revoke the standby's new grant by declaring all-stop for itself.

Recommend explicit operator-selected acquisition initially. Automated primary-to-standby transfer can later be a declared policy using this sequence; it is not inferred from `+`. External manual override/failsafe inhibits the affected routine path and requires explicit reconciliation/reacquisition under the selected policy, rather than an automatic attempt to recapture control.

Grant renewal is a separate bridge decision with defined issuer, evidence and expiry. Fresh intent can use an existing grant but does not implicitly renew it. Producer restart changes the incarnation even if its configured app name is reused. Native engine restart within one process also needs a fresh producer binding because iteration identity can restart. [E3](evidence.md#e3-restart-and-rejournal).

**Still open:** actual priority table, automatic-transfer eligibility, operator interfaces, lease duration/renewal policy, external override observation and acceptable residual interval. No claim of ownership over external RC/GCS paths follows from a bridge grant.

## 4. Egress and residual work: Q-AUTH-04

**Recommendation:** define the enforceable boundary before claiming a deadline or clean transfer. An architecture diagram with an “authority manager” is insufficient if a plugin, SDK retry or proxy can send after that manager revokes permission.

The prior study shows asynchronous library/transport queues and command retries, while pArduBridge's sender loop can interleave queued work with setpoint sends. Those are reasons to inspect selected paths, not proof that every queue is uncancellable. Core MOOS also has finite pending queues; its outbox can discard old unsent entries. [E5](evidence.md#e5-core-moos-messages-mail-and-retention), [P1](evidence.md#p1-prior-bridge-and-library-evidence).

For each effectful path, later architecture/library evaluation must record:

| Boundary | Required question |
| --- | --- |
| Admission/snapshot selection | Is the whole immutable object current and permitted? |
| Local work queue | Are source/vehicle/authority generations, deadline and cancellation status attached? Can stale work be coalesced or removed? |
| SDK command helper/retry | Who owns retries, what deadline do they obey, and can they outlive a revoked scope or bridge session? |
| Last controllable dequeue/write | Can permission/session/deadline be checked with a defined ordering relative to revocation? |
| OS/driver/link buffers and transit | What maximum work/delay can remain after that check? Is the bound measured/configured/unknown? |
| FC receipt and continued execution | Does the FC recognize an application generation? What release/timeout semantics are actually qualified? |

A gate and the handoff it authorizes need a clear linearization/barrier rule. Checking permission and then releasing a lock before an unbounded later send is not a demonstrated post-revocation guarantee. Whether a later implementation uses one owner, immutable snapshots, generation checks or cancellation tokens is still open.

Proposed narrow claim: after the revocation boundary, no **newly authorized** routine sends can be created under that epoch, and controllable old queued work is invalidated. Bytes already handed off remain residual work. To claim that no old target can arrive after time T, establish all remaining queue/retry/transit bounds or a downstream epoch mechanism that actually enforces it. Ordinary application envelope IDs do not make the FC interpret those IDs when the selected MAVLink payload lacks them.

Do not put an invented residual value into the profile. If the chosen dependency cannot expose a sufficiently late gate or bounded drain, the profile must change its guarantee, choose another integration, or use a qualified transition strategy. Navigation timing should have a budget independent of potentially blocking transactions, but no thread/process layout is selected here.

**Still open:** full path inventory, library selection, cancel/drain behavior, miss handling and measured maximum residual. This pass defines the question and proposed obligations; it does not establish a bound for an unselected stack.

## 5. Vehicle identity and continuity: Q-SH-02

**Recommendation:** distinguish provisioned target identity, advertised physical identity, and session continuity evidence. A MAVLink system/component ID or network address can identify a routing target without proving unique hardware or unchanged boot state.

The first profile should provision its expected target/ingress association and report the strength of available evidence. Where persistent hardware identity or boot evidence is unavailable, label that limitation. It is reasonable to create a new logical vehicle session conservatively on target/link continuity ambiguity; that is a protective policy, not proof that the hardware rebooted.

| Change | Proposed handling |
| --- | --- |
| Bridge process restart | New bridge epoch; old windows/grants cannot resume. Re-evaluate vehicle continuity before creating/rebinding context |
| Target reconnect, endpoint reuse or conflicting identity/boot evidence | New vehicle session unless the chosen profile can establish continuity; reacquire affected authority |
| Producer/decision-engine restart | New producer incarnation and ingress binding; do not infer a new physical vehicle solely from that event |
| MOOSDB connection loss/reconnect | Invalidate affected navigation permission/ingress according to profile and require a new exchange; do not assert FC reboot from DB loss |
| Home/origin/estimator reference reset | Invalidate affected reference-dependent intent/observations, even if physical identity and vehicle session remain otherwise known |

Do not let a “same vehicle” assertion restore old permission. Physical continuity and authorization continuity are separate. Conversely, avoid making every reference change an undocumented claim that a different vehicle exists; invalidate the affected references explicitly.

**Still open:** FC-specific identity/boot evidence, handling of redundant links and ID reuse/wrap, exact continuity policy, and operator-visible evidence strength. No additional autopilot identity implementation was traced in this pass.

## 6. Restart and uncertain effects: Q-SH-04

**Recommendation:** reset effectful admission bindings on bridge restart and keep a separate account of effects that may already have escaped. This avoids replay without claiming to undo a command.

For navigation, a new bridge epoch invalidates prior grants/windows/queued objects. Routine streaming resumes only after a new exchange, current observations and explicit acquisition. The vehicle may still be executing its last target; independent FC/release behavior must be assessed before an automatic transition.

For discrete transactions, distinguish at least these crash positions:

| Crash position | What can be concluded afterward |
| --- | --- |
| Before any possible effectful handoff, with trustworthy retained evidence | Local record can indicate not handed off; future action still needs new authorization/current policy |
| After possible handoff, before a durable handoff/result record | Effect is uncertain; lack of a stored success record cannot prove failure |
| After ACK but without fresh state/task confirmation | Protocol evidence exists at its documented strength; physical/task result remains separate |
| After observed state predicate | The recorded as-of predicate may have held; it does not establish causal attribution or present state after restart |

A proposed durability model must say what is written before the first possible side effect, when that write becomes durable, and how callbacks/retries interact with crash recovery. Persisting “done” only after a callback cannot close the handoff-to-record gap. Writing “will send” beforehand also does not prove that sending occurred.

At minimum, do not make old ingress/session objects admissible after forgetting their dedup ledger. Either retain enough evidence for their entire admissible lifetime or invalidate those bindings before forgetting. A new unpredictable/unique bridge epoch plus a required new exchange can prevent old retained objects from becoming new work, provided all old ingress paths are rejected. It does not recover lost history of a possibly executed command.

If durable evidence is unavailable, expose recovery uncertainty and require an explicit reconciliation policy for affected effectful scopes. Read-only current state queries are useful; automatic retransmission of an uncertain non-idempotent action is not a generic reconciliation rule. A state such as “armed” can satisfy a present-state goal without proving which command caused it. The selected operation contract must state whether and how that permits settlement.

**Still open:** durable store choice/resource limits, crash model, restart epoch allocation, handoff ordering, ledger retention/compaction, history access and per-operation reconciliation. No exactly-once physical execution guarantee is proposed.

## 7. Carry-forward into the next contract draft

If adopted, the eight focused recommendations should appear together: trusted producer binding, producer-complete outcomes, pre-start origin windows, explicit invalidity, bridge-owned authority generations, conservative session loss, final egress gates and explicit recovery uncertainty. Choosing only a new envelope format would leave most of the guarantee unestablished.

The next architecture review should fill ranks 9–17 as one concrete execution profile. Command correlation/result/deduplication must be completed before any discrete effectful path is added. See [priorities](priorities.md) and [paper cases](review-cases.md).
