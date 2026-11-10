# Parallel capability implementation batch

November 4, 2026. Baseline de7e30c. The user approved the proposed three-stream implementation and equal model/reasoning settings. Stock pHelmIvP is unchanged; no real vehicle control is authorized.

## Work and acceptance

- Copter: add an explicit operation interface for supported native, Guided-target and configured stock-helm strategies; preserve legacy command/NAV inputs. Demonstrate go-to/hold/return, cancellation/takeover/recovery, facing options and native RTL in owned SITL. Unsupported combinations reject explicitly.
- Missions: independent bounded mission download/upload/readback/clear/progress provider. Exercise loss, duplicate/reorder, original expiry, reset and concurrent navigation. Mutation is restricted to a disarmed owned simulator; data transfer never starts execution. Fence/rally are later extensions of the family, not silently advertised.
- Rover: independent ArduRover control provider and two-dimensional helm contract, own modes/identity/holding semantics, explicit manual takeover and owned Rover SITL. Copter height/airborne assumptions cannot become Rover prerequisites.
- Root: shared interfaces, authority/lifetime, bounded protocol lane, profile-specific simulator guards, build/app integration, Linux/software/sanitizer/native regression and archived evidence. Three disjoint Git worktrees isolate feature work.

## ABI 6 interface freeze

`NavigationTarget` keeps its four legacy scalar members, adding explicit `kind` (CourseSpeed/GlobalPosition), `dimensions` (Spatial/Planar), global latitude/longitude and `facing` (Course/Keep/Heading with heading degrees clockwise from north). `height` is metres above home only when Spatial; Planar has no altitude objective. The original `expires` bounds each local send proposal. Persistent operation strategies may issue new short-lived proposals only while their original operation deadline and current authority remain valid; helm values keep their original publication expiries.

`VehicleControlBackend::helmProfile()` declares required height/home and dimensions. Existing Copter defaults remain three-dimensional. Frame construction and readiness/modes remain provider policy; the host retains final time/authority checks.

Mission effects use typed `MissionRequest` and `MissionItem` values through `Services::mission`, with phase `id`, stable `operation_id`, original `operation_expires` and phase `expires`. The host owns encoding/fixed addresses and a bounded lane with final send checks, then calls `Plugin::onMissionSent`. `Services::missionFinished` retires queued phases/mutation leases for that operation. The mission protocol permits one active operation and at most four queued phases. Read-only requests need fresh heartbeat; Count/Item/Clear additionally require disarmed/non-navigation/non-command state, with Copter ground evidence. Mission mutation blocks host control acquisition/commands until terminal/expiry; external state change aborts further mutation. No rollback or recall of transmitted data is claimed.

`Services::navigationGranted()` exposes navigation permission and `controlBusy()` reports native command transactions only, excluding a mission client's own lease. Both are observational. `cancelCommand(id)` retires waiting external work without retiring sent-effect uncertainty; `commandFinished` retires the lease separately. `projectGlobal(lat,lon,x,y)` delegates to the single configured telemetry projection owner (MOOSGeodesy, x east/y north), with no freshness renewal or effect. Vehicle horizontal estimator validity is separately available so Rover can require mask 19 without inheriting Copter vertical eligibility. These methods grant no authority. All services default-deny when not implemented by a fake host. Factories must rebuild against ABI 6.

## Integration and evidence

The integrator alone edits shared app/runtime/build files. Agents commit feature-only changes in their own worktrees; integration uses reviewed commits and focused tests, then combined regressions. Every effectful run is an owned localhost simulation with scoped cleanup. Full flight tests exceed 30 seconds because GPS/EKF convergence and flight/landing are observed at 1x time; each wrapper keeps explicit deadlines. Preserve failures separately from corrected reruns, and distinguish fake I/O, loopback, MOOS, Linux core, full Linux app and native simulator evidence.

Status: implemented and qualified at the archived software/native layers. Final macOS software44/44 and ASan/UBSan44/44 pass; clean source-built aarch64 Linux43/43 passes; fifteen native simulator scenarios are accepted, including final Copter operations, disarmed mission transfer, mission/parameter reads during helm flight, Rover and eleven legacy regressions. Actual transfer outcomes, test-only corrections, original failures and exact source/artifact identity are in the implementation ledger. Mission normalization/retry/reset/fault cases remain fake-Service coverage unless separately recorded; native upload uses HOME plus two WAYPOINTs. Stock pHelmIvP is unchanged, and real vehicle control remains excluded.
