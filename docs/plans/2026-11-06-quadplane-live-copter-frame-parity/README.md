# QuadPlane live helm and Copter frame parity

November 6, 2026. The control provider owns both the selected vehicle's native FC operations and its supported stock-helm guidance. Native operations remain distinct from helm acquisition, and both go through the same host authority and final-send gates. The previous checkpoint qualified native QuadPlane missions and ordinary Copter hexa and octa lifecycle. This campaign extends those exact pinned configurations. Prior qualification

## QuadPlane

On pinned ArduPlane 4.6.3, the standard fixed-motor QuadPlane supports a native AUTO flight across both phases. In Guided, the ordinary point command uses the fixed-wing approach path. With `Q_GUIDED_MODE=1`, the aircraft hovers once it reaches the point, and receiving a new point can send it back into fixed-wing flight. The position-target messages handle altitude. Live stock helm therefore uses the stable forward phase with verified `Q_GUIDED_MODE=0`. Native QLoiter and the verified native mission provide the tested hover, transition, and landing operations. Continuous hover helm would require a different FC-side interface. [ArduPilot Guided and AUTO behavior](https://ardupilot.org/plane/docs/quadplane-auto-mode.html#guided-mode) · [Plane Guided MAVLink commands](https://ardupilot.org/dev/docs/plane-commands-in-guided-mode.html) · [Pinned Guided target setter](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/commands.cpp#L71-L110)

- Keep live helm explicitly opt-in. Preserve native-only configurations, exact SITL identity, and mutual exclusion with a retained AUTO execution or recovery marker.
- Reuse conventional fixed-wing point translation, addressed readback, and the watchdog handback policy where their contracts agree. Verify the exact airframe, firmware, phase and FC parameters before admitting live control. Count observed target readback and measured stock-helm movement separately.
- Require fresh acquisition after abnormal loss, takeover or phase change. A short ordinary helm all-stop may resume within its original window. Reject acquisition in hover or transition, invalidate work queued for a previous phase, and observe native Loiter handback after a prolonged interruption. Exercise the shared authority fence while native AUTO owns flight.
- Preserve the complete native takeoff, transition, QRTL, landing, disarm, and handback workflows. Record the firmware's phase sequence independently and retain useful negative evidence.

## Copter

Qualify the standard hexa plus and octa plus layouts on the same pinned Copter 4.6.3 release as the existing quad. Frame identity and defaults stay explicit. The command surface and all `arducopter_control` features should match the quad for these named layouts.

- Compare the actual configured capability catalogs. Exercise legacy and versioned native lifecycle, direct/Guided and configured stock-helm operations, relative adjustments, local home observation, facing choices, and the scoped dead-man return.
- Measure Precision Loiter capture of an offset beacon, its dwell, and release of the behavior after disable while the target is still detected. Treat addressed command acceptance and physical capture as separate evidence. [ArduPilot Precision Loiter](https://ardupilot.org/copter/docs/precision-landing-and-loiter.html)
- Fly the typed stored mission from verified upload through takeoff, same-cursor pause/resume, waypoint dwell, actual landing, disarm, and explicit handback. Exercise stale proof, takeover, cancellation and process-restart ownership boundaries. Preserve the older untyped airborne route checks.
- Share the existing quad fixtures and runtime logic where the contracts match. The separate observation, payload and protocol providers keep their own qualification scopes.

## Execution and evidence

Build in isolated worktrees against the existing immutable dependencies. Serialize native runs in the macOS lease. Use the exact isolated Linux domain for any separate parallel native run. Protect the stock helm binary and simulator identity, retain raw independent wire and onboard observations in compact checksummed archives, and reclaim duplicate staging after replay. Select the affected loaded-provider software tests in local CI. Every reported firmware and frame result keeps its own source and artifact identities.

Qualification and replay
