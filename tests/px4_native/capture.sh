#!/bin/sh
# Runs only inside the isolated, exact-image PX4 capture container.
set -eu
export PATH="/opt/px4/bin:$PATH"
cd /evidence
cat /proc/net/dev > interfaces
cat /proc/net/route > ipv4-routes
cat /proc/net/ipv6_route > ipv6-routes
cat /proc/net/if_inet6 > ipv6-addresses
cat /proc/self/status > process-status
for flags in /sys/class/net/*/flags; do
  net=${flags%/flags}
  printf '%s %s\n' "${net##*/}" "$(cat "$flags")"
done > interface-flags

px4_pid=
capture_pid=
cleanup()
{
  for child in "$px4_pid" "$capture_pid"; do
    if [ -n "$child" ]; then
      kill "$child" 2>/dev/null || :
      wait "$child" 2>/dev/null || :
    fi
  done
}
trap cleanup EXIT
trap 'exit 1' HUP INT TERM
/fixture/capture_udp > recorder.log 2>&1 &
capture_pid=$!
for attempt in 1 2 3 4 5 6 7 8 9 10; do
  [ ! -f capture-ready ] || break
  sleep .1
done
test -f capture-ready
px4 -d /opt/px4/etc -w /evidence -s /opt/px4/etc/init.d-posix/rcS > startup.log 2>&1 &
px4_pid=$!
for attempt in 1 2 3 4 5 6 7 8 9 10; do
  if grep -q 'Startup script returned successfully' startup.log; then break; fi
  kill -0 "$px4_pid"
  sleep .25
done
grep -q 'Startup script returned successfully' startup.log
px4-ver all > version.log
sha256sum /opt/px4/bin/px4 > firmware-binary.sha256
tr '\000' ' ' < "/proc/$px4_pid/cmdline" > px4-cmdline
cat /proc/net/udp > udp-sockets
for fd in "/proc/$px4_pid/fd/"*; do readlink "$fd" || :; done > px4-descriptors
wait "$capture_pid"
capture_pid=
kill -0 "$px4_pid"
