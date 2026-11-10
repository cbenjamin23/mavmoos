# Durable control requests

The iMAVLink host records discrete control request IDs before admitting work that can change vehicle state. An ID already recorded for the route returns `DUPLICATE` or is refused by the provider's current state guard. Retrying the same ID after a bridge restart cannot start the action again. The record is a reservation. A crash after reservation and before socket handoff can consume the ID without a packet reaching the FC.

The stock helm's live setpoints still use freshness, ownership and final-send checks. Managed missions reserve the operator request once and can then send the required sequence of phases under their existing deadlines. The AUTO recovery marker separately records the possibility that the flight controller still owns the mission. Neither mechanism proves execution on the FC.

By default the active app creates a private journal next to its generated mission file. The journal is scoped by MOOS community, app alias, provider profile and expected FC identity. Keep that directory and scope when restarting a route. If mission files are regenerated in another directory, set `control_replay_directory` to an existing absolute private directory and reuse that path. A former Rover `plugin.ardurover_control.native_replay_directory` setting selects the same host journal and preserves its existing ID records. The replay and AUTO recovery directories are distinct.

`MAVMOOS_CONTROL_REPLAY_STATUS` reports availability, occupancy and capacity. The journal holds 4,096 IDs. Full, locked, invalid or mismatched storage blocks fresh protected requests. The journal never silently evicts IDs. The operator needs an independently defined replay horizon before resetting a route's request history.

ArduSub accepts `MAVMOOS_SUB_HANDBACK=schema=1,id=surface-001,action=surface` and the same form with `action=hold`. Legacy `hold` and `surface` messages and Copter dead-man events use their original MOOS publication identity. Reposting their text as a new publication creates a new request. Use an explicit ID when retrying a Sub request across process restart.

Validation uses the shared journal's process-death tests, loaded-provider software tests and owned SITL. Each native receipt identifies its exact source, stock MOOS-IvP and firmware. The phase transitions in a managed request can legitimately use multiple MAVLink messages. Integrated results and crash witnesses

When adding a discrete control request, verify the journal through the loaded provider, including any service adapter. A test that constructs the journal directly cannot catch a provider that fails to forward it.
