# Evidence and provenance

October 29, 2026. Source/documentation inspection only. The 52-file manifest records immutable URLs, Git blob/SHA-256 hashes, byte/line counts, branch-head resolution metadata and inspection methods. Target firmware remains explicitly unselected.

## Baselines and retrieval

| Project | Files | Provenance |
| --- | --- | --- |
| `mavlink/MAVSDK` | 25 | Exact-commit GitHub connector fetches; bytes matched returned blob SHAs at `8b1ad926a879a72aefe0e66f606e0349f8186f28` |
| `mavlink/mavros` / libmavconn | 11 | Existing source snapshot at `2d3f4db1101c7950c8817a9d788f0d11106fb261`; selected bytes matched re-fetched exact-commit Git tree blob SHAs |
| `Scarter668/MAVSDK` | 8 | Local pArduBridge dependency checkout at `e8ff09a5dada80ccded46e28df0395c917d2d997`; selected bytes matched Git |
| `cbenjamin23/moos-ivp-uav-base` | 2 | Local pArduBridge bytes matched `e34a62ef49c2be42215a69c351850cbd85881486` |
| `mavlink/c_library_v2` | 4 | Exact-commit GitHub fetches at `a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29`; matching blob SHAs |
| `mavlink/mavlink-devguide` | 2 | Exact-commit GitHub fetches at `b6d83e20c397f05e634986d400ddb1b42385fa19`; matching blob SHAs |

MAVSDK main, MAVROS ros2, C-library master and guide master were resolved October 29. The MAVROS head matched the earlier study. These are research snapshots, not stable-release recommendations or selected dependencies. The fork remains a separate historical baseline; its synchronous sends must not be described as current upstream behavior. Selected source was inspected without building/running/installing it. Existing checkout changes were preserved.

Official rendered pages consulted for context: [command protocol](https://mavlink.io/en/services/command.html), [C library use](https://mavlink.io/en/mavgen_c/) and [current MavlinkDirect API](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_mavlink_direct.html). Source traces use the pinned bytes rather than relying on a moving API page. Exact source snapshots were retained in task scratch space for inspection; the repository archives findings and provenance, without integrating third-party implementation code.

## Finding map

| Finding | Primary evidence in the traces |
| --- | --- |
| Deferred callbacks and per-field age are separate; heartbeats republish cached health | [Ingress section](source-traces.md#1-ingress-and-observation-age): current/fork telemetry heartbeat handlers, current EKF and callback queue paths |
| Typed position conversion omits that report's boot time; broad in_air differs from exact IN_AIR | Same section: current telemetry global-position and extended-state handlers |
| libmavconn exposes raw/framing evidence, but no receive timestamp | Same section: interface callback and parse loop |
| SDK command work has its own retries, queue/timer policy and no inspected per-request deadline/cancel contract | [Acquisition section](source-traces.md#2-guided-acquisition-and-command-retries): both command senders, MavlinkDirect and Passthrough command paths |
| ACK fields lack a MAVMOOS generation nonce; COMMAND_CANCEL does not purge local queues | Same section: generated ACK record and pinned command guide |
| SDK Offboard changes stream/cadence/yaw/mode policy; raw-message methods are a different option | [Egress section](source-traces.md#3-navigation-egress-and-queues): Offboard setter/sender, mode map and raw packing/send path |
| Current SDK posts sends; serial/TCP add bounded queues, while UDP's inspected path sends synchronously | Same section: current send/deliver, three connection implementations and TxQueue |
| libmavconn FIFO has capacity checks but no per-work TTL, selective cancel or public per-frame receipt | Same section: public API and UDP/serial/TCP insertion/write chains |
| An SDK interceptor is a useful gate with association/downstream/result limits; forwarding can bypass it | [Gate section](source-traces.md#4-sdk-interceptors-forwarding-and-version-differences): public hook declarations, common delivery and forwarding |
| Standalone library organization is different from qualified standalone packaging/platform support | Same section: libmavconn README/CMake/package metadata |

All source-span links in those sections refer to files in this manifest. Missing mechanisms are conclusions limited to the inspected public interfaces/write paths; no claim is made that a vendor extension or different revision could never supply them.

## Inference and validation boundary

The preference for host-owned policy, libmavconn-first transport evaluation, direct-I/O alternative, conservative one-wire mode-attempt clarification and proposed evidence/settlement interface are **design recommendations inferred from the comparison**. They are not existing MAVMOOS features, accepted decisions, proof of SDK/library defects or demonstrated reliability improvements.

Source inspection can identify queue locations, retry producers, missing metadata and API boundaries. It cannot establish wall-clock scheduling, thread/race correctness of a new adapter, network/kernel/driver bounds, FC command timing, successful cancellation, physical capture or stopping behavior. The profile's timing/controller/envelope questions remain open. No firmware commands, parameter writes, SITL/hardware trials, build/dependency scaffolding or executable missions were created or run.

## Documentation checks completed

At creation, all 52 source/documentation records passed Git/blob byte and hash checks. Repository-local links/headings and immutable source-span bounds passed; the 12 backend criteria and 14 paper-case IDs were unique/complete. Twenty-one checked historical files remained byte-identical, including every dated ArduCopter draft-01 file. The README remained within its medium-length target. These are provenance/documentation checks, not executions of the paper scenarios or validation of a backend.
