# Water-depth observations

Select `water_depth` at app startup to observe the bound ArduPilot surface boat at FC system and component 1/1. The provider publishes `MAVMOOS_WATER_DEPTH` with capability `water_depth.water_depth`. It has no request owner, stream request, navigation authority or outbound operation. Source identity requires a current ArduPilot `MAV_TYPE_SURFACE_BOAT` heartbeat. The supported producer is the Rover boat branch traced below.

## Source and measurement

The generated MAVLink 2 ArduPilot dialect defines `WATER_DEPTH` as message 11038, CRC extra 47, with a 38-byte payload before legal trailing-zero truncation. The pinned Rover producer checks boat frame class and a downward-facing rangefinder that has data, then sends one instance per call. The health byte is 1 only when the rangefinder reports `Good`. Out-of-range readings can be sent with health 0, while disconnected and no-data sensors are skipped. The sender copies uncorrected distance and vehicle attitude, samples FC boot milliseconds while sending, copies vehicle location without checking its success and uses zero temperature when the backend has no temperature. [Generated MAVLink header](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/ardupilotmega/mavlink_msg_water_depth.h) · [MAVLink field definitions](https://mavlink.io/en/messages/ardupilotmega.html#WATER_DEPTH) · [Pinned Rover sender](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/Rover/GCS_MAVLink_Rover.cpp#L191-L257) · [Rangefinder status and data](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/AP_RangeFinder/AP_RangeFinder_Backend.cpp#L46-L68)

The published bottom distance is along the sensor beam. Corrected vertical depth, chart datum and automatic avoidance need separate measurement and qualification.

## MOOS record

The schema 1 envelope carries `family=WATER_DEPTH`, `message_id=11038`, `read_only=true`, `authoritative_for_control=false` and `atomic=false`. Its `samples` array retains the newest accepted frame independently for each observed sensor ID 0–9. The limit follows the pinned SITL rangefinder instance bound. Every sample exposes the actual MAVLink 2 payload length and its own source, session, original local receipt time and age metadata. `sample_timeout` defaults to 1 second and `publish_interval` to 0.1 second. A stale observation retains its original sample with `fresh=false`. A source or session boundary clears the slots. [Pinned rangefinder instance bound](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/AP_RangeFinder/AP_RangeFinder.h#L30-L36)

| Per-sensor data | Meaning |
| --- | --- |
| `time_boot_ms_raw` | Unsigned FC wire timestamp. The pinned sender timestamps transmission, while local metadata tracks bridge acquisition. |
| `health_raw`, `health` | Raw byte and `healthy`, `unhealthy` or `unknown`. |
| `distance_m_raw`, `bottom_distance_uncorrected_m`, `distance_status` | Finite signed raw distance and optional beam distance. The optional value requires health 1 and a finite nonnegative reading. The status distinguishes reported healthy, unknown/nonfinite, negative or unavailable health. |
| `latitude_e7_raw`, `longitude_e7_raw`, `latitude_deg`, `longitude_deg`, `position_status` | Raw integers and geographic degrees for an in-bounds pair other than (0, 0). A decoded pair remains `reported_unconfirmed` because the source omits a location-success bit. Both-zero and out-of-bounds pairs leave the decoded fields null and keep the raw integers. |
| `vehicle_altitude_msl_m_reported`, `vehicle_roll_rad_reported`, `vehicle_pitch_rad_reported`, `vehicle_yaw_rad_reported` | Reported vehicle MSL altitude and attitude in radians. Each nonfinite value becomes null. |
| `temperature_c_raw`, `temperature_c`, `temperature_status` | A finite nonzero temperature preserves its sign. Raw zero is retained and decoded temperature is null with `zero_ambiguous`, since the sender uses it for unavailable temperature. Nonfinite temperature becomes null with `unavailable_nonfinite`. |

A current unhealthy sample can be fresh. That describes when the bridge received it, while the data status describes the reading. Structurally invalid frames and out-of-bound sensor IDs increment `invalid_samples`. Validly framed unknown fields are published with their explicit availability and preserve the remaining reported values.

## Evidence

The component DSO witness exercises the generated healthy and unhealthy boat examples, exact source and profile checks, all-zero MAVLink 2 tail truncation, original age, recovery and invalid values. The app can reuse `tests/water_depth/WaterDepthFrames.h` and `tests/water_depth/audit.py` with a boat-shaped RX_ONLY fixture. That fixture uses expected vehicle type 11 and a passive provider roster.

The pinned firmware's `DepthFinder` autotest supplies a native simulator blueprint. It uses the serial NMEA backend as a downward sensor with temperature, a forward I²C sensor which the producer excludes and a downward I²C sensor without temperature. The [optional disarmed boat witness](../../tests/water_depth/NATIVE.md) uses that source setup and passed on the pinned disarmed motorboat. Exact evidence. [Pinned ArduPilot DepthFinder autotest](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/Tools/autotest/rover.py#L6038-L6110)
