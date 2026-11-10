# Copter SITL, stock pHelmIvP

One local MOOS community: `MOOSDB`, `iMAVLink`, stock `pHelmIvP`, `pLogger`, and `uProcessWatch`. The FC simulator is a separately owned native ArduCopter process. There is no marine simulator/PID chain and no real-vehicle route. Launch scripts fix time warp at 1 and expose MOOS and UDP ports for isolation.

`meta_vehicle.bhv` uses stock Waypoint and StationKeep behaviors plus the existing normal `BHV_ConstantAltitude` extension from `moos-ivp-uav-base`. This uses stock helm's public behavior-loading mechanism; it does not change the helm or its scalar publication contract. Supply `--behavior_dir` if the selected library lives elsewhere. The standalone launcher defaults to a Canberra origin, speed1.5m/s and height10m. The automated flight test explicitly uses the Boston origin42.358456/-71.087589, speed1m/s and height5m. Keep mission origin and simulator home paired when changing either.

Build MAVMOOS first, put the built stock MOOS-IvP `bin` directory on PATH, then generate shadow-mode targets:

```sh
./launch.sh --just_make --nogui --veh_mport=19001 \
  --app=/absolute/build/bin/iMAVLink --plugin_dir=/absolute/build/plugins \
  --behavior_dir=/absolute/moos-ivp-uav-base/lib
```

`./launch.sh --nogui` launches the default shadow mission. It proposes/explains navigation but never enables effectful transmission. Specify all paths for a non-default build. The app itself remains receive-only by default; this mission explicitly enables its helm adapter in shadow mode.

The effectful simulation form additionally requires `--simulation-only --sitl_pid=PID`, where PID belongs to the native ArduCopter process launched for this run, and matching `--bind_port`/`--peer_port` endpoints. A loopback address alone does not establish ownership; the app's simulation guard must qualify that process/route. The automated workflow, its exact firmware and evidence are documented separately under `docs/implementation/sitl/`.

Operator sequence: establish fresh simulator telemetry; request Guided, then arm and takeoff through the command plugin; wait for confirmation; poke `DEPLOY=true`, `MOOS_MANUAL_OVERRIDE=false`, then `MAVMOOS_NAV_CONTROL=acquire`. The helm supplies course, speed and home-relative height. Route completion switches to StationKeep while maintaining altitude. Request an explicit hold or landing through the command plugin. To release navigation, poke `MAVMOOS_NAV_CONTROL=release`. Request/result syntax follows the command module's interface documentation.

Use `WPT_UPDATE` for an ordinary waypoint update and `ALTITUDE_UPDATE=altitude=10` to update the altitude behavior. These do not alter the helm's output contract. The bridge labels separately published fields `ARRIVAL_GROUPED`; no atomic solver-generation guarantee is claimed.

The top-level launcher owns the interactive `uMAC` session. `--xlaunched` instead waits in the foreground for an owned automated wrapper to manage timeout and scoped teardown. `clean.sh` removes generated targets and logs only. There are no shoreside/pShare port options because this is one community.
