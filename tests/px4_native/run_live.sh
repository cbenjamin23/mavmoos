#!/bin/sh
# Execute inside one offline ARM64 container with the frozen /work and /opt/px4 volumes.
set -eu
app=/work/mavmoos-passive-build
moos=/work/moos-ivp
export PATH="$app/bin:$moos/bin:/opt/px4/bin:$PATH"
export PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim
cd /evidence

cat /proc/net/dev > interfaces
cat /proc/net/route > ipv4-routes
cat /proc/net/ipv6_route > ipv6-routes
grep -E '^(CapEff|NoNewPrivs):' /proc/self/status > process-status
test "$(wc -l < ipv4-routes)" -eq 1
awk '$NF != "lo" {exit 1}' ipv6-routes
grep -Eq '^CapEff:[[:space:]]+0+$' process-status
grep -Eq '^NoNewPrivs:[[:space:]]+1$' process-status
for flags in /sys/class/net/*/flags; do
  net=${flags%/flags}
  value=$(cat "$flags")
  printf '%s %s\n' "${net##*/}" "$value"
  if [ "${net##*/}" != lo ]; then test "$((value & 1))" -eq 0; fi
done > interface-flags
sha256sum "$app/bin/iMAVLink" "$app/plugins/mavmoos_telemetry.so" "$app/plugins/mavmoos_odometry.so" \
  "$moos/bin/MOOSDB" "$moos/bin/pHelmIvP" /opt/px4/bin/px4 > artifacts-before.sha256
grep -Fxq "$EXPECTED_APP_SHA256  $app/bin/iMAVLink" artifacts-before.sha256
grep -q '^23f0b5eb07430868b5525a5c59e28185fe5b0da35ade9a7e04bdea5894144ea8 ' artifacts-before.sha256
core=$moos/MOOS/MOOSCore/Core/libMOOS
moos_build=$moos/build/MOOS/MOOSCore
set --
for includes in "$core" "$core"/* "$core"/Thirdparty/* "$moos_build/Core/libMOOS"; do
  if [ -d "$includes/include" ]; then set -- "$@" -isystem "$includes/include"; fi
done
c++ -O2 -std=c++17 -pthread "$@" /fixture/Probe.cpp "$moos_build/lib/libMOOS.a" -lm -o observer_matrix_probe
cc -O2 -std=c11 -Wall -Wextra -Werror -DPX4_GCS_ONLY /fixture/capture_udp.c -o capture_udp
sha256sum /fixture/Probe.cpp observer_matrix_probe /fixture/run_live.sh \
  /fixture/capture_udp.c capture_udp > probe.sha256

cat > px4-live.moos <<'MOOS'
ServerHost = 127.0.0.1
ServerPort = 9000
Community = px4_live_observer
MOOSTimeWarp = 1
TERM_REPORTING = false
ProcessConfig = iMAVLink
{
  AppTick = 50
  CommsTick = 50
  bind_address = 127.0.0.1
  bind_port = 14540
  peer_address = 127.0.0.1
  peer_port = 14580
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 12
  simulation_only = false
  shadow_mode = false
  plugins = telemetry,odometry
  plugin_directory = /work/mavmoos-passive-build/plugins
}
MOOS

db_pid=
app_pid=
probe_pid=
px4_pid=
capture_pid=
cleanup()
{
  for child in "$px4_pid" "$capture_pid" "$probe_pid" "$app_pid" "$db_pid"; do
    if [ -n "$child" ]; then
      kill "$child" 2>/dev/null || :
      wait "$child" 2>/dev/null || :
    fi
  done
}
trap cleanup EXIT
trap 'exit 1' HUP INT TERM
MOOSDB px4-live.moos > moos.log 2>&1 &
db_pid=$!
./observer_matrix_probe 9000 24 > probe.log 2>&1 &
probe_pid=$!
sleep .4
iMAVLink px4-live.moos > imavlink.log 2>&1 &
app_pid=$!
for attempt in 1 2 3 4 5 6 7 8 9 10; do
  if grep -q MAVMOOS_RUNTIME_STATUS probe.log; then break; fi
  sleep .25
done
grep -q MAVMOOS_RUNTIME_STATUS probe.log
./capture_udp > recorder.log 2>&1 &
capture_pid=$!
for attempt in 1 2 3 4 5 6 7 8 9 10; do
  [ ! -f capture-ready ] || break
  sleep .1
done
test -f capture-ready
px4 -d /opt/px4/etc -w /evidence -s /opt/px4/etc/init.d-posix/rcS > px4-startup.log 2>&1 &
px4_pid=$!
sleep 14
kill -0 "$px4_pid"
px4-ver all > version.log
px4-mavlink status > px4-mavlink.log
cat /proc/net/udp > udp-sockets
for role in app px4; do
  pid=$app_pid
  if [ "$role" = px4 ]; then pid=$px4_pid; fi
  tr '\000' ' ' < "/proc/$pid/cmdline" > "$role-cmdline"
  for fd in "/proc/$pid/fd/"*; do readlink "$fd" || :; done > "$role-descriptors"
done
grep "$app/plugins/mavmoos_" "/proc/$app_pid/maps" > app-module-maps
kill "$px4_pid"
wait "$px4_pid" || :
px4_pid=
wait "$capture_pid"
capture_pid=
wait "$probe_pid"
probe_pid=
kill -0 "$app_pid"
sha256sum -c artifacts-before.sha256 > artifacts-after.log
