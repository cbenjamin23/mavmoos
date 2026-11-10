# Event timeline

`event_timeline` is an optional passive observer for the bound primary flight controller. It follows one heartbeat-qualified source on an ArduPilot- or PX4-shaped route and shares the host connection. Its three capabilities are observations with zero request owners and zero observation-stream requests.

```moos
plugins = telemetry,event_timeline
plugin.event_timeline.sample_timeout = 5
plugin.event_timeline.publish_interval = .1
```

The defaults allow for the Events protocol's nominal three-second sequence advertisement. The common passive observer accepts timeouts of 0.25–10 seconds and publish intervals of 0.02–2 seconds. The interval must be at most the timeout. Each received record retains its original local acquisition times and age basis.

| MOOS variable | MAVLink message | Observation |
| --- | --- | --- |
| `MAVMOOS_EVENT_TIMELINE` | `EVENT` (410) | Last 16 selected events in acceptance order, with sequence position and continuity counters |
| `MAVMOOS_EVENT_SEQUENCE_STATUS` | `CURRENT_EVENT_SEQUENCE` (411) | Most recent advertised sequence, raw flags, unknown bits and reset disposition |
| `MAVMOOS_EVENT_REPLAY_ERROR` | `RESPONSE_EVENT_ERROR` (413) | Most recent observed error, raw target and sequence bounds, and the reported reason |

Every event record retains the raw event ID and namespace byte, boot-time milliseconds, sequence and destination. The internal and external log levels retain their raw nibbles and decode names for 0–9. `arguments_raw_hex` holds all 40 decoded argument bytes. `argument_wire_bytes` identifies the bytes present in the MAVLink 2 payload before trailing-zero restoration. Event text and typed argument meanings belong to the sender's external metadata. [MAVLink Events interface](https://mavlink.io/en/services/events.html)

Broadcast events and events addressed to the bridge's system 245/component 191 enter the timeline. Valid events addressed to another recipient still advance the sequence and increment `other_destinations`. Their event contents stay out of the timeline. This follows the protocol's broadcast routing for sequence accounting. [MAVLink event delivery and addressing](https://mavlink.io/en/services/events.html#sending-events)

Sequence comparisons use unsigned 16-bit wraparound. For example, a change from 65534 to 0 records one wrap and one unobserved event at the jump. The timeline retains that event's `missing_before` and `sequence_wrap`, and accumulates `holes_observed` and `sequence_wraps` within its current sequence epoch. Repeated latest, older and half-range-ambiguous sequences have separate counters. `evicted` counts local ring retention, independently of sequence gaps.

An initial sequence advertisement supplies a baseline when no event has arrived. Later advertisements report the FC's sequence independently of received events. `advertised_unseen` measures how far the last advertisement is ahead of the current observed position. A behind or half-range-ambiguous advertisement yields a null distance. An error records the reported requested sequence and oldest available sequence with `request_correlation:"none"`. It leaves the observed sequence position intact.

A reset flag starts a new sequence epoch, clears the old timeline and replay-error observation, and sets a new baseline. Consecutive flagged reports carrying the same sequence count as one reset. Records with acquisition times predating the applied reset are fenced out. Heartbeat loss, local clock failure and host reset also clear the source cache. The event boot timestamp stays raw through these changes. The bridge tracks original local age, with the remote event age marked unknown.

The wire definitions are pinned in the generated library. [EVENT, blob `814c0c0`](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_event.h) · [CURRENT_EVENT_SEQUENCE, blob `974b18a`](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_current_event_sequence.h) · [RESPONSE_EVENT_ERROR, blob `efb518a`](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_response_event_error.h)

The component fixture exercises the loaded module, source and time gates, wrap, gaps, duplicate and old events, explicit reset, replay errors, argument padding and capacity. `EventFrames.h` provides five generated packets per advancing batch for an app witness. `wire_fixture.py` checks exact payloads, original ages, link loss and rebound in the retained probe output.
