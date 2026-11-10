# Qualifying control commands and transitions

See the [command and transition coverage matrix](control-coverage.md) for the evidence and gaps for each control provider.

The [capability registry](../../config/capabilities/copter.json) identifies the current command families and their required software witnesses. The filename remains the path used by existing receipts. The control providers exercise real command parsing and state transitions in software. Native SITL then establishes the effects that depend on a particular flight controller, frame and configuration.

For an effectful command, tests distinguish request acceptance, local packet handoff, a matching protocol response and a fresh independent mode or motion observation. A command can pass by refusing a conflicting request. Every acquisition needs fresh permission and evidence. Check that a stale reply, resumed helm mail or a later mode report cannot restore a retired grant. At send time, check that expiry, changed references and pilot takeover prevent a pending packet.

## Supported command and transition coverage

The test names below are required software CTests unless marked native. Each provider guide gives the original request lifetimes and configuration switches.

| Request path | Implemented command forms and transition checks | Main witnesses |
| --- | --- | --- |
| [Copter native commands](../plugins/arducopter_control.md) | Legacy `guided`, `arm`, `disarm`, `takeoff`, `hold`, `land`, `rtl` and their versioned native forms (`return` in place of `rtl`). The software matrix checks both paths, explicit height and both omitted-height rules, ACK ordering, and fresh mode, armed or landed evidence. Native Loiter retires helm, and fresh acquisition from Guided is separate. | `copter_parity`, `command_plugin`, `copter_operations`, native `copter_sitl_workflow` and `copter_native_operations` |
| Copter adjustments and configured actions | Legacy `change_speed`, `reset_speed_min`, `change_course`, `precision_loiter_on/off` and configured dead-man RTL. Test that adjustments reject during held helm authority, and that a previous speed cap expires on leaving Guided. Precision Loiter also depends on the preflight parameter reads. | `copter_parity` and the stable variant of native `copter_native_operations`. The native precision case measures beacon capture, dwell and disabled hold from onboard evidence. Dead-man RTL has independent mode and landing evidence. Core ledger |
| Copter versioned Guided, helm and local operations | Guided `goto`, `hold` and `adjust_altitude`. Guided positions support course, keep-yaw and explicit heading. Helm `goto`, `hold`, `return`, configured `survey` and `voronoi`, `adjust_speed`, `reset_speed_min` and `adjust_altitude`. Local `cancel` and `visualize_home`. Test malformed, stale and busy requests and rejected strategy combinations such as native goto and Guided return. | `copter_operations`, `stock_helm_operations`, native `copter_native_operations`, route and wind cases. The stable fixture checks survey and Voronoi selection with stock waypoint stand-ins. Helm speed reset and facing combinations have software evidence. |
| Copter stock helm and managed AUTO | Explicit helm acquire/release, fresh course/speed/height and both bounded normal handoff and expiry. Verified AUTO confirms cursor and actual mode and supports `pause`, typed-plan same-cursor `resume`, stop-only `abort` and observed `release`. Held or uncertain AUTO fences competing native, Guided, dead-man and helm requests. Restart uses separate current-session handback with fresh non-AUTO evidence. | `helm_plugin`, `resume_permit_review`, `authority_review`, `copter_parity`, `copter_mission_execution`, `mission_execution_runtime`, `auto_recovery_core`. Native cases cover operator release, helm faults, typed mission touchdown, pause/resume, pause/abort and restart/recovery. |
| Standard Copter hexa and octa | Match the quad control surface for the named plus layouts. Require exact heartbeat identity, native frame evidence, matching catalogs, physical Precision Loiter, typed AUTO and safe restart with explicit fresh helm. | `copter_parity`, `mission_execution_runtime`, `copter_mission_execution`, `auto_recovery_core`. Advanced frame parity · Restart evidence |
| [Ground Rover and boat](../plugins/ardurover_control.md) | Legacy Guided, arm, disarm, Hold and RTL. Versioned native equivalents, plus boat-only Loiter in both forms. Guided goto, zero-speed hold and return, cancellation and planar helm acquire/release. Software checks the ground rejection of both Loiter forms, mode/arming observation after ACK, velocity masks and stopped dwell, direct-to-helm ownership changes, busy denial and takeover just before send. | `rover_backend`, `rover_control`, `rover_operations`, `rover_profile_guards`. Native `native_rover_ground` and `native_rover_boat` cover the separate frames, all-stop expiry, Manual, link loss and reacquisition. Boat Loiter and its powered hold have a distinct native witness. |
| Rover and boat native missions | Verified disarmed waypoint storage, AUTO start, stopped pause, same-cursor resume, arrival/completion, stop-only abort/release, Manual takeover and retained recovery across restart. Reentering AUTO after pilot takeover keeps the old abort fenced. | `ardu_rover_mission_runtime`, `rover_backend`, `rover_control`, `rover_operations`. Exact ground and powered-boat cells · AUTO reentry |
| [Conventional Plane](../plugins/arduplane_control.md) | Verified runway takeoff, stock helm and configured tasks, direct waypoint/orbit and adjustments, native Loiter/RTL, aligned 4.6.3 landing or 4.7.2 AUTOLAND, and observed disarm/Manual release. | `plane_backend`, `plane_control`, `plane_runtime`, `plane_helm_tasks`. Exact 4.6.3 and 4.7.2 runway cells · Stock tasks · Earlier takeover and loss |
| [Vectored Sub](../plugins/ardusub_control.md) | Stock-helm depth acquisition, captured hold, native Position Hold and Surface. Surface requires native ascent and a shallow Depth Hold dwell. Test pre-send cancellation, wrong source, timeout, Manual and permission retirement. | `ardusub_control`. Stable 4.7.2 Surface passed for vectored and vectored-6DOF at `d41e81b`. The earlier stable dive/loss cell retains its own source. |
| [PX4 quad](../plugins/px4_control.md) | Stock course/speed/height acquire/release, anchored prestream, bounded pause and continuity loss. Opt-in native arm, disarm, takeoff, Hold, Return and Land. Native completion separates addressed ACK from fresh effect. | `px4_runtime`. New 1.17.0 lifecycle and [earlier airborne loss/recovery](../implementation/2026-11-06-stable-px4/README.md) retain their own sources. Configured yaw variants have software evidence. |
| [PX4 fixed-wing](../plugins/px4_control.md#conventional-fixed-wing-workflow) | Native lifecycle and stock course, nominal speed and height. Verify forward prestream and handoff, MSL lookahead, exact masks, airspeed, position ordering, intended mode and final-send cancellation. | `px4_fixed_wing_runtime`. Native lifecycle and independent onboard position targets. The [earlier airborne runner](../../tests/px4_offboard/FIXED_WING.md) covers loss and reacquisition separately. |
| PX4 standard VTOL | Native lifecycle, hover and forward stock-helm strategies, both native transitions and denied acquisition during transition. Phase change or stale phase retires the previous grant and target. | `px4_runtime`. Exact standard SIH VTOL cell includes hover/forward/hover and independent onboard command/phase/setpoint replay. |
| [ArduPilot QuadPlane](../plugins/arduquadplane_control.md) | Verified native VTOL plan, QRTL and QLoiter handback, and separate forward and hover helm configurations. Check phase, original expiry, proof-read configuration, and no old target crossing takeover or a native AUTO fence. | `quadplane_runtime`, `plane_runtime`. Native forward, AUTO and hover evidence |
| [PX4 Ackermann](../plugins/px4_rover_control.md) | Experimental opt-in exact SIH profile. Forward planar helm acquisition and release, bounded handoff, native mission start/pause/resume/abort/release and explicit restart handback. Test stale mode or GNSS, configuration mismatch, expired heartbeat, conflicting requests and Manual just before handoff. | `px4_rover_backend`, `px4_rover_mission`, `px4_rover_runtime`. [Owned Ackermann runner](../../tests/px4_rover/README.md) exercises actual steering, loss and stop, mission completion, stop-only abort, release and retained Mission across restart. Final-send and link fault injection have separate software witnesses. |

The examples combine actual variants with faults at the points where ownership can change. Native receipts describe their observed case and firmware, and the registry keeps declarations and test selections separate from archived qualifications.


## Current expanded evidence

Each completed native cell below has its own source and physical assertions. The campaign ledger links the exact native and software receipts.

| Workflow | Current evidence | Scope |
| --- | --- | --- |
| Copter standard hexa and octa | Both advanced operator, typed AUTO and restart cells PASS | Pinned ArduCopter 4.6.3 plus layouts. Frame parity · Restart and motion |
| Rover ground and powered boat native missions | Both original missions and pilot reentry reruns PASS | ArduRover `04cf6486` with separate stopped/Loiter contracts. Original mission qualification · Reentry qualification |
| Conventional Plane runway and Guided workflows | PASS `qualification-463-04` at `8a6128e` and `qualification-472-02` at `463b5e2` | Distinct AUTO landing and native AUTOLAND release paths, each with observed ground/disarm/Manual release. Archive and replay |
| Conventional Plane nudges and configured stock tasks | PASS at `35f40fd35b1d457325e7f9f10ad1441626f2b6d0` | Physical course and altitude change, stock return/survey/Voronoi, cancel, expiry and FBWA takeover. Exact task evidence |
| PX4 quad, airplane and standard VTOL lifecycle | PASS at `c7a33bee3604710b6e2457aaf4620ddd01475d7c` | Three exact PX4 1.17.0 SIH models reran after publication-based expiry. Independent GCS/onboard replay |
| Copter bounded takeoff, waypoint dwell, pause/resume and landing plan | PASS at `d5429b8377d63902c53ce041fd93164fd692ce70` | Exact quad typed-plan lifecycle and final landing cursor guard. Required `copter_mission_execution` and `mission_execution_runtime` also retain the original waypoint form. |
| ArduPilot QuadPlane normal, QRTL, takeover and restart | All four dedicated native cells PASS | One exact 4.6.3 fixed-motor QuadPlane with phase-aware mission authority. Adverse native evidence |
| ArduSub native Surface | PASS at `d41e81b1aff74039af300532de9ab2f521e4a410` | Stable 4.7.2 vectored and vectored-6DOF ascent, permission retirement and native Depth Hold. [Owned fixture](../../tests/ardusub/run_native.py). |
| Current local release | PASS 24/24 at `06625eb` | 165 selected software checks, 22 native SITL cases and a relocated Linux kit. Exact results |

The registry's 21 legacy machine-verified qualification cells use their original quad-only evidence schema. New declaration rows link these expanded capabilities to existing required software tests. The linked ledgers carry exact cross-profile native source, artifact and cleanup evidence. All 34 v1 provider milestones retain their original denominator. The current build adds PX4 Ackermann and ArduPilot QuadPlane for 36 providers, including seven control providers.

## Running the gates

- `scripts/ci/run.py --tier software` builds the configured checkout and runs all required software checks. `--tier focused --case ctest.copter_parity` selects exact software IDs for fast feedback.
- `--tier full` adds the existing full native ArduPilot matrix. With the verified Copter 4.6.3 quad pin, `ctest.copter_native_operations` runs the parity variant and requires its command and helm proof. Other firmware profiles retain their matched base scenario.
- `--tier focused-native` selects one or more exact required native IDs with the same pinned inputs, supervision and postflight checks as the full tier. Use it to run the higher-risk cases after a change without claiming that the full tier passed. Examples include `ctest.copter_native_operations`, `native.copter.auto_complete_handback`, `native.copter.auto_pause_abort`, `ctest.auto_restart_recovery_sitl`, `ctest.fault_command_timeout`, `ctest.native_rover_ground` and `ctest.native_rover_boat`.
- The [release campaign](sitl-release.md) owns the cross-profile cases for Copter, both Rovers, Plane, Sub and PX4. The [Plane runner](../../tests/plane_native/README.md) and stable PX4 and Sub receipts retain their own firmware pins and native cases.
- Run [Ackermann full, restart and abort](../../tests/px4_rover/README.md) in the separately leased Linux domain. They are dedicated native cells. The local software tier always requires the three Ackermann checks.
- The [PX4 aircraft lifecycle runner](../../tests/px4_offboard/AIRCRAFT.md) reuses one sealed build for quad, airplane and standard VTOL. The Copter frame runner, [Rover mission runner](../../tests/rover_mission/run_native.py) and [Plane runway runner](../../tests/plane_native/README.md) retain distinct manual native receipts.

See the local CI guide for the exact native inputs. Run native cases serially in the host domain. A failed run keeps its evidence and uses a fresh directory on retry.

With the existing pinned Copter 4.6.3 build, the parity and pause/abort cases can be selected together:

```sh
python3 -B scripts/ci/run.py --tier focused-native --build "$BUILD" \
  --output "$FRESH_OUTPUT" --jobs 1 \
  --firmware-manifest config/firmware/arducopter-4.6.3-quad.json \
  --firmware-attestation "$COPTER_ATTESTATION" --ardupilot-root "$COPTER_SOURCE" \
  --moos-root "$STOCK_MOOS" --behavior-dir "$ALTITUDE_BEHAVIOR_DIR" \
  --sitl-python "$FIXTURE_PYTHON" \
  --case ctest.copter_native_operations --case native.copter.auto_pause_abort
```

For Rover, configure the build with its separate `MAVMOOS_ROVER_ARDUPILOT_ROOT` and select the matching ground or boat case ID. The shared local runner verifies both the selected Copter manifest and the configured Rover binary identity. The Rover fixture determines the model, profile and expected MAVLink type.
