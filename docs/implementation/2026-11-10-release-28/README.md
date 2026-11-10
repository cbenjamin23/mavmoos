# Full 28-scenario release gate

The local November 9–10 campaign passed all 30 stages at source revision `fa07e5ec8fc2f8f8e2413996ce606438d2f9173b`. All 165 software checks passed, including 162 required checks. The Linux ARM64 install check loaded all 36 standard modules from a relocated installation. All 28 SITL scenarios passed on their first attempt.

The six required stress missions covered Copter, Rover boat, conventional Plane, QuadPlane, ArduSub and PX4 standard VTOL. Each passed its independent evidence replay. PX4 Ackermann remains an experimental case with its own qualification outside this gate.

The campaign used the pinned upstream MOOS-IvP revision `023fb54e00692611ecee2cd6f1138d10eec2127e`. Its source tree and the `pHelmIvP` binary matched the preflight check after the run. All vehicle activity was local simulation.

The private archive lives beside the checkout at `mavmoos-evidence/2026-11-10-release-28-fa07e5e.zip`. It includes the release plan, package, run logs, original receipts and replay evidence. Each of the 2,350 files was verified by SHA-256 from a fresh extraction. Running the release verifier on that extraction reproduced the original passing report byte for byte. The [readback receipt](evidence/readback.json) records the archive checksum and complete case list.
