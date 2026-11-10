# 0018: Observations and final handoff checks

Date: November 4, 2026. Status: implemented continuation within the user's authorized simulator-only plugin and robustness work; qualification is recorded in the dated ledger.

## Context and implementation choice

The user authorized clear further work while asleep and encouraged parallel agents. The next bounded batch adds useful observations and strengthens existing effect boundaries. It adds no motion mode, mission execution or hardware route. Stock pHelmIvP and default receive-only behavior remain unchanged.

The optional `system_status` provider groups system sensor masks, identified batteries, primary raw GPS and FC diagnostic text. It has no request owner, command service or control backend. Observations do not automatically become control-readiness gates. Telemetry retains conventional NAV projection ownership; control providers retain native/Guided/helm strategies. MAVROS's `sys_status` and raw-GPS handling inform this division; exact source comparisons and deliberate differences are archived with the provider.

Development ABI 7 passes `MessageTime{effective_at, read_at, age_basis}` through an observation callback. The default callback preserves existing protocol clients' actual read-time correlation. The effective time can be the conservative socket-empty acquisition floor; it is not a remote generation timestamp. Older ABI modules must be rebuilt. This is a matching-build development interface, not a stable third-party binary promise.

The real sender accepts an original absolute expiry and a live host eligibility predicate. It checks time before and after native endpoint proof, invokes that predicate, checks time again, then makes one nonblocking UDP attempt. Connection revisions prevent an earlier request surviving a continuity reset even when new telemetry has already made the current state fresh. Each command, navigation target, recovery hold, mission phase and parameter request retains its own original bound and ownership semantics. Housekeeping uses a short local lifetime; it grants no navigation authority.

## Alternatives and consequences

Receipt-only observer dispatch would lose the already-established socket-queue age bound. Treating effective acquisition as protocol receipt would change ACK correlation; the distinct callback avoids that. Hiding system observations inside a vehicle-control provider would couple diagnostics to motion ownership. Automatically deriving a safety verdict from sensor masks, battery estimates or GPS fix would exceed this feature's evidence.

Checking time only before a potentially slow endpoint proof leaves a local stale-send window. The new sequence narrows that gap while preserving exact PID/socket/profile validation. It does not make checks atomic with the OS syscall, guarantee scheduling latency or recall packets already sent. UDP and the MAVLink wire protocol are unchanged.

Mission cancellation remains local retirement, with explicit possible remote mutation. The owned native tests distinguish an immediate rejected read while the FC still owns its transfer, later partial-storage readback, and a fresh-ID clear. A local cancel does not issue an abort/rollback or prove that an already-transmitted item was ignored. Producer timestamps added to mission send diagnostics allow evidence to order accepted sends against terminal results without assuming MOOS cross-variable delivery order.

## Evidence and remaining questions

The batch ledger identifies component, integration, sanitizer, Linux and native evidence separately, with source/artifact hashes and preserved failures. [System status](../plugins/system_status.md) documents units, sentinels, cache/text bounds and acquisition/session metadata. Numeric defaults and the tested FC build are not vehicle-wide timing or physical-sensor guarantees. Broader battery/GPS variants, extra observation load during concurrent flight, hardware links and mission execution remain separate qualification/design work.
