# 0012: Socket queue age, owner continuity and operator diagnostics

Date: November 3, 2026. Status: accepted continuation scope; numerical defaults and interface details are engineering choices under the user's instruction to keep going. Stock pHelmIvP remains unchanged, receive-only remains default, and all effectful validation remains owned local simulation.

## Problem and evidence

The previous receiver timestamped a datagram immediately after reading it. A stopped or overloaded app could therefore treat a packet that had already waited in the UDP receive queue as newly fresh. An immutable `943ff39` reproduction queued a position for 650.710 ms and then reported it FRESH with an age of 11.4 microseconds. Bounded datagram counts alone did not bound that waiting time. See ingress evidence.

The app also refreshed vehicle observations before its ordinary runtime tick. New traffic could hide the expiry of previously required observations during an owner-loop interruption. Independent review separately reproduced command completion/ACK rescue at or after transaction deadlines. These are demonstrated component findings, not physical vehicle incidents.

## Choices

1. **Use one conservative socket-queue policy on both operating systems.** Sample monotonic time before each nonblocking `recvmsg`. An EAGAIN result proves the socket queue was empty at an instant no earlier than that sample. Retain that lower bound; do not renew it merely because a read succeeded. The initial bound is sampled before binding the new socket. For later reads, `read_time - empty_floor` bounds waiting since socket-queue admission/visibility from above. It says nothing about pre-socket, network or source-generation delay.
2. **Drain stale epochs with bounded work.** Default `max_queue_age=0.25` s, configurable 0.01..2 s. Once a read exceeds the bound, discard queued packets until a new EAGAIN proves emptiness. Keep the existing maximum datagrams/iteration and maximum datagram bytes; there is no unbounded catch-up loop. Continuous overload can prevent recovery and discard newly arriving traffic as well as old traffic. This is an explicit liveness tradeoff, visible in status counters, rather than silent freshness renewal.
3. **Separate observation time from protocol receipt.** Raw/NAV schema 2 identifies `age_basis=QUEUE_EMPTY_FLOOR`, the conservative effective time, the actual read time and whether age is an upper bound. Vehicle eligibility uses the effective time. Protocol callbacks use actual read time: a valid reply's conservative floor can predate a request sent between polls. Parameter replies still have no request identifier and cannot prove causal association with a particular send.
4. **Check the old cache before incoming updates.** The owner calls `Runtime::checkpoint(now)` before mail/UDP ingress; the runtime also checkpoints direct message/mail entry points. Old prerequisite expiry revokes NAV before a new observation can conceal the gap. Heartbeat/session loss retains the existing full reset; a position-only gap does not cancel an independent parameter read. New `Plugin::onInputBoundary()` provides expiry/latching checks without starting or sending work. The helm checks Active/Paused continuity there; ordinary iteration alone activates, resumes or proposes targets.
5. **Advance the development ABI to 3.** The lifecycle callback and vehicle timing metadata require matching rebuilt host/modules. This is not a stable external C++ ABI promise. Module loading remains startup-only and trusted in-process code.
6. **Make deadlines win over newly delivered evidence.** The command module expires pending work before processing ACKs or calculating new completion. Equality is expired. Timely ACK or previously observed progress can permit longer takeoff/landing execution under the unchanged operation deadline. Late input cannot rescue a terminal transaction, and timeout never proves that the FC did not execute a command.
7. **Explain recovery without performing it automatically.** Append phase-aware readiness, blockers, input ages/lifetime, acquisition preparation and advisory next-action fields. Missing scalars in a deliberately inactive cache do not tell the operator to wait forever for inputs the module is ignoring. LOITER may be ready for explicit acquisition while remaining ineligible for navigation. Module-local readiness is explicitly separate from host gates and actual FC authority.

## Alternatives considered

macOS exposes a monotonic ancillary timestamp; Linux commonly exposes realtime receive timestamps. A tighter OS timestamp policy would require verified clock conversion, clock-step handling and platform-specific tests. The portable empty-queue bound avoids those assumptions for this increment. OS timestamps remain a possible refinement, not a requirement. Increasing the kernel buffer or retrying indefinitely would hide age and incomplete transfers rather than solve them.

A loop-gap threshold alone would miss observations already near expiry before the gap. Checkpoints instead evaluate the previously observed prerequisites at current owner time. Running every plugin's full iteration twice would risk duplicate effects; the new boundary hook performs only continuity/expiry work.

## Transferable architecture

The queue policy belongs to transport ingress; vehicle state consumes explicit time semantics; each protocol owns its transaction deadline; the helm owns its producer/handoff state. These boundaries retain MAVROS's shared context/feature organization while addressing MOOS-specific helm semantics. No MAVLink packet format, UDP extension, helm hook or upstream edit is introduced.

## Evidence and remaining limits

Batch ledger, operator guide, ingress tests and independent review/Linux evidence separate immutable baseline, candidate and final builds. Source/network age, hard scheduling bounds, remote recall, general radio behavior and hardware qualification remain unproven. Consumers still need their own watchdog because MOOSDB retains a status string while the app is stopped.
