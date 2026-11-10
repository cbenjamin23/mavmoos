# 0025. External command authoring and durable AUTO uncertainty

November 5, 2026. Implemented for owned native SITL under the [bounded increment plan](../plans/2026-11-05-command-recovery-stable/README.md). The three streams keep the existing startup-loaded provider layout and matching development ABI 9.

## External command reference

The optional [standalone provider](../../examples/disarmed_command/README.md) selects one non-forced DISARM on an already disarmed, on-ground Copter. It exercises an effectful transaction without a change in flight state. The [bench](../../missions/external_command/README.md) loads the regular Copter backend with native, helm and AUTO request facets disabled. The example owns one MOOS request surface and uses the same typed host admission, clock, handoff and cancellation services as standard modules. It keeps one active operation and retires it by the original deadline. Local handoff, ACK and a separate post-handoff state observation have separate outcomes. A cancelled request whose frame already reached the transport can remain uncertain.

Keep the example outside the ten standard modules. It uses the exact matching host headers, generated MAVLink and trusted process-resident plugin ABI. Extend the established typed host service when an actual new capability needs it. Effectful modules use the shared vehicle link and scheduler.

## AUTO restart

An FC can continue executing in AUTO after the bridge dies. A local lease alone loses that uncertainty. The host now creates and synchronizes a presence-only marker in an existing owner-private, exclusively locked directory before admitting the operation. The owned Copter SITL configuration uses the same recovery directory across a bridge restart. Retain that path when disabling new starts for the same simulator. Preserve the marker during uncertainty. The startup and handback flow is in the [operator interface](../plugins/copter_auto.md#recover-after-an-imavlink-restart).

A new process accepts no previous proof, grant, deadline or executable intent. If the marker exists or the FC is observed in unowned AUTO, the host blocks motion, competing native commands, parameter or mission mutations and stream interval changes. Heartbeats and permitted reads can continue. The operator takes the owned FC out of AUTO through its existing control path, then posts a fresh explicit handback keyed to the current process session. The bridge requires fresh non-AUTO evidence before clearing and synchronizing the marker, resetting local work and allowing a new explicit acquisition. Storage faults keep the fence closed. The local filesystem contract covers bridge restart on the same machine; use the same configured private directory.

This increment leaves emergency LAND/RTL priority, automatic recovery motion and a broader field control policy for separate decisions. Stock pHelmIvP continues to use its ordinary interfaces.

## Firmware evidence

The selected stable release is ArduCopter 4.6.3, source `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`, using the [pinned quad profile](../../config/firmware/arducopter-4.6.3-quad.json). Rover uses its independent source and fixture. Each result belongs to the tested source, artifact, profile, platform and scenario, and previous results retain their original identities.

The complete macOS ARM64 matrix passed 133 of 133 cases at `119f723`. The standalone consumer, two restart targets, original interrupted attempts, and independent replay are recorded with their own exact scopes. Implementation and qualification ledger
