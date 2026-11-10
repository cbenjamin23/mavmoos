# 0006: Prefer an owned UDP candidate for the narrow first topology

October 29, 2026. **Status: Proposed; conditional on the first deployment topology.** No backend/dependency/platform selection or implementation authorization follows.

## Context

[Decision 0005](0005-host-owned-navigation-egress.md) proposes host-owned eligibility, navigation scheduling and acquisition retry policy, initially recommending libmavconn transport evaluation. The [queue assessment](../research/2026-10-29-udp-queue-assessment/README.md) now compares a full managed FIFO, a smaller queue-bypass extension and an owned UDP path against the existing Copter draft. Earlier records/profile remain unchanged.

The library's async UDP operation borrows FIFO storage. Selective invalidation therefore needs stable active ownership and result/fence semantics, not just erase/TTL fields. A smaller bypass can keep policy in MAVMOOS, but still needs an exclusive managed write path, controlled peer/RX evidence, packing ownership and a matched vendor/package integration.

## Proposal

If the first deployment fits one configured IPv4 UDP peer and one Copter, prefer a narrowly owned UDP transport as the next backend architecture candidate. Reuse generated MAVLink and suitable I/O primitives; keep vehicle/helm/authority/transaction policy and a bounded ManagedEgress responsibility shared in the one-app design. No cached-target repeater or hidden command retries belong to the transport.

Retain the UDP-only libmavconn FIFO-bypass extension as the closest reuse alternative. Defer a full managed async-FIFO retrofit until its transport breadth/benefit justifies the larger active-operation/cancellation surface. Neither serial/TCP qualification nor an immediate live FIFO-to-managed transition is included in this first candidate scope.

## Rationale and tradeoffs

Most policy work is common to the owned and bypass options. An owned UDP path avoids maintaining a vendor write/peer/RX patch and gives an explicit immediate-send result boundary, while taking responsibility for a narrow socket/parser/signing/lifecycle integration. A libmavconn path reuses existing facilities and may be preferable if required transport breadth, an existing compatible build, discovery or routing changes the scope. No quantitative cost/reliability advantage has been measured.

A local fence means matching old write initiation is blocked after its serialized application; local drain and remote reconciliation are separate facts. Already accepted data is not recalled. The proposed attempt/acceptance timing interpretation needs explicit review before claiming the Copter draft's candidate budgets.

## Revisit conditions and evidence

Revisit when the first platform/link/peer topology is known, especially for serial/TCP, multiple peers, discovery, forwarding, or required dependency reuse. Resolve packing/signing ownership, bounded admission/event delivery, peer/receive limits, handoff/fence semantics and downstream residual behavior before accepting a backend or implementation scope.

The assessment's [17-file evidence](../research/2026-10-29-udp-queue-assessment/evidence.md), [queue/fence sketch](../research/2026-10-29-udp-queue-assessment/queue-and-fence-design.md) and [14 paper cases](../research/2026-10-29-udp-queue-assessment/direct-udp-and-review.md) support this conditional recommendation. All capabilities remain Candidate. Stock MOOS-IvP/pHelmIvP remains unchanged under [accepted decision 0004](0004-use-stock-moos-ivp.md).
