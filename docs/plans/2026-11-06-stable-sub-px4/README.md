# Stable ArduSub and PX4 SITL qualification

November 6, 2026. Qualify ArduSub 4.7.2 and PX4 1.17.0 against the existing vectored Sub and SIH multicopter release cases. Use the pinned stock pHelmIvP and the frozen MAVMOOS binaries from release source `4eb4e295acdbfd7f119040da49e4847d6e8dc337`. The baseline records the prior firmware artifacts and independent observations. [Official Sub stable artifacts](https://firmware.ardupilot.org/Sub/stable-4.7.2/) · [Official PX4 stable tag](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0)

| Cell | Preserve the baseline behavior checks |
| --- | --- |
| ArduSub vectored | Begin in native Position Hold, acquire Guided, and drive with fresh stock course, speed and depth. Measure motion and captured hold after the helm falls silent. Verify that resumed helm mail leaves the retired grant inactive. Observe native handback and Depth Hold after bridge heartbeat loss, then prove Manual takeover and final disarm. |
| PX4 SIH quad | Begin airborne in native Hold. Verify bounded prestream and observed Offboard entry, measured stock-helm motion, retirement after silence and independent native Hold. Check the unsafe reacquisition denial while Offboard intent survives, explicit native handback and fresh reacquisition, then landing and disarm. |

Record the exact stable source, simulator binary, MOOS/helm, bridge, loaded modules, fixture and platform identities. Use the same authority checks and measurement limits. A firmware-specific setup change must be explicit and preserve the corresponding observation. Record both the original and corrected attempts when a fixture needs adjustment. A failed behavior check remains a failed qualification for that pin.

All execution uses owned loopback SITL. Keep ArduSub on the common native host lease and PX4 in its isolated Linux namespace with the local Docker and ownership checks. Reuse existing inputs where possible and archive the original wire and onboard logs, candidate receipts, checksums and teardown evidence.

Alongside the firmware runs, reconcile the 34 MAVMOOS providers with the current official MAVROS ROS 2 catalog. Separate observed workflow confidence from API breadth and identify uncovered capabilities and their scope decisions.
