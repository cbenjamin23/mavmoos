# Evidence and provenance

October 29, 2026. Static source/documentation analysis only. The 13-file manifest records exact revisions, SHA-256/Git blob hashes, bytes and line counts. Proposed profile values are distinguished from observed constants; no controller, timing, vehicle response or firmware compatibility was validated.

## 1. Baselines

- **pArduBridge:** `~/moos-ivp-uav-base`, commit `e34a62ef49c2be42215a69c351850cbd85881486`. Nine selected source/configuration files matched committed bytes. Its MAVSDK gitlink is `e8ff09a5dada80ccded46e28df0395c917d2d997`. This is reference provenance, not a selected MAVMOOS dependency or identification of the user's actual FC binary.
- **Stock helm:** MOOS-IvP commit `023fb54e00692611ecee2cd6f1138d10eec2127e`; the selected HelmIvP.cpp bytes matched the commit. Existing [stock compatibility analysis](../../design/2026-10-29-built-in-modules/stock-helm.md) supplies source/manual semantics and retained-mail limits.
- **ArduPilot documentation:** official rendered pages consulted October 29 and their RST sources fetched at `ArduPilot/ardupilot_wiki` commit `23c3dcd846a92aefe7eaf584f222d8bec91fb095`. This pins documentation, **not firmware**. Target release/configuration are explicitly unselected.

Unrelated source-checkout changes were preserved. No dependency/source installation, source modification, mission, SITL, hardware connection, command or firmware parameter change occurred.

## 2. pArduBridge source map

| Finding | Primary source at the pinned revision |
| --- | --- |
| Three helm scalar inputs and local receipt-age implementation | [ArduBridge.cpp 124–138](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp#L124-L138); [SetpointManager.h 44–115](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/SetpointManager.h#L44-L115) |
| Default two-second input watchdog | [ArduBridge.cpp 43–55](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp#L43-L55) |
| Airborne flag from relative altitude; threshold is five metres | [UAV_Model.cpp 579–583](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.cpp#L579-L583); [definitions.h 17–20](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/definitions.h#L17-L20) |
| Reported landed state already exists separately | [UAV_Model.cpp 597–603](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.cpp#L597-L603); [UAV_Model.h 245–254](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.h#L245-L254) |
| Mode-first handoff and later confirmation | [ArduBridge.cpp 2035–2062](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp#L2035-L2062); [1515–1559](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp#L1515-L1559); [ModeConfirmationTracker.h 25–65](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ModeConfirmationTracker.h#L25-L65) |
| General mode-change allowlist includes more than this draft | [UAV_Model.cpp 889–952](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.cpp#L889-L952) |
| Copter velocity/altitude/yaw mapping and no routine Guided recapture | [ArduBridge.cpp 1562–1588](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp#L1562-L1588); [UAV_Model.cpp 1317–1403](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.cpp#L1317-L1403) |
| Relative altitude source; health/landed/confirmation constants and five-Hz default telemetry | [UAV_Model.h 49–58](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.h#L49-L58), [138–145](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.h#L138-L145), [270–275](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.h#L270-L275) |
| Stale-input latch and current-position hold scheduling; Land/RTL exclusions | [ArduBridge.cpp 1473–1496](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp#L1473-L1496) |
| Reference position helper requests Guided and encodes relative-home altitude plus yaw | [UAV_Model.cpp 1537–1639](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.cpp#L1537-L1639) |
| Sender sleeps after queued work; nominal 100 ms is not a bound | [UAV_Model.cpp 2111–2155](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.cpp#L2111-L2155) |

The reference mission plug requests AppTick/CommsTick 4; it does not prove a ten-Hz host-owner loop. The proposed profile's navigation scheduler and 0.30 s local-gap budget require separate host/backend feasibility work. The [earlier command/setpoint traces](../../research/2026-10-28-operating-models/README.md) supply the broader call paths and SDK result limits.

## 3. Official protocol and configuration context

**Movement messages.** The [official Copter movement guide](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html) documents the supported NED/global target forms, masks, axis conventions and resend expectation. Its [pinned RST source](https://github.com/ArduPilot/ardupilot_wiki/blob/23c3dcd846a92aefe7eaf584f222d8bec91fb095/dev/source/docs/copter-commands-in-guided-mode.rst) is recorded in the manifest. This supports checking the proposed active velocity packet and position-only MSL capture representation; it does not qualify their effects on a particular aircraft. The message timestamp description is sender-system uptime, not a proof of helm origin or a universal requirement to use the FC's boot epoch.

**Guided options/timeout.** The [official Guided page](https://ardupilot.org/copter/docs/ac2_guidedmode.html) identifies configurable Guided options and `GUID_TIMEOUT`. Its [pinned source lines 85–125](https://github.com/ArduPilot/ardupilot_wiki/blob/23c3dcd846a92aefe7eaf584f222d8bec91fb095/copter/source/docs/ac2_guidedmode.rst#L85-L125) distinguish normal velocity navigation, controller-option changes and Guided_NoGPS. The three-second default described there is not a measured response for this draft or a substitute for reading the actual vehicle setting. The initial profile assumes native XY stabilization and requires review of the option mask before activation.

**Independent GCS failsafe.** The [official GCS failsafe page](https://ardupilot.org/copter/docs/gcs-failsafe.html), [pinned source lines 1–32](https://github.com/ArduPilot/ardupilot_wiki/blob/23c3dcd846a92aefe7eaf584f222d8bec91fb095/copter/source/docs/gcs-failsafe.rst#L1-L32), describes heartbeat-source/configuration-dependent behavior and reconnection that does not automatically restore the previous mode. The draft therefore separates helm silence, vehicle-link loss, process crash and native failsafe transitions. It does not select a failsafe parameter action.

## 4. New choices and validation boundary

Negative-speed rejection, narrow LOITER/GUIDED acquisition, reported IN_AIR navigation gating, sample-backed mode confirmation, added publication/spread/observation/queue limits, the absolute-MSL/yaw-ignored capture plan and latched explicit recovery are **new draft proposals**. They are not findings that those mechanisms already exist or have passed tests in pArduBridge.

Paper arithmetic can verify masks, signs and the distinction between independent timers. It cannot establish acceptable gain, yaw response, braking/capture behavior, source-age truth, clock error, observed rate, firmware mode-entry behavior, external authority, queue cancellation or physical stopping time. Those gaps remain explicit in [loss/review](loss-and-review.md#7-still-open-before-execution-qualification).

## 5. Documentation checks completed

On October 29, all 13 manifest records passed hash/byte/line-count verification against the pinned local Git bytes or fetched wiki blobs. Documentation checks passed for local links/headings, source-span bounds, the 15 unique paper-case IDs, target-mask arithmetic (2503/3527/3576), course-to-velocity signs and the illustrative independent-timer arithmetic. Historical discussion/contract/source-trace files remained unchanged. These are source and document checks only; the 15 expected-outcome cases have not been exercised against a bridge, simulator or vehicle.
