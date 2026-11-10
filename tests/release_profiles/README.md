# Release vehicle qualification

Run these selected native scenarios against the same clean MAVMOOS commit used by the release campaign. The profile launcher supervises each existing fixture and checks the resulting source, executable, stock helm and firmware identities. It retains the fixture's original report and raw observations and binds them to a common receipt. Evidence covers the specific pinned build, model and observed outcome.

| Case | Native gate | Platform and frame |
| --- | --- | --- |
| rover_ground | Stock helm travels both axes, observed Manual takeover, Guided expiry after link loss and explicit reacquisition | Darwin ARM64, ArduRover revision 04cf6486, rover |
| rover_boat | The ground gates plus active native Loiter under the modeled tide and explicit reacquisition | Darwin ARM64, ArduRover revision 04cf6486, motorboat-skid |
| plane_normal | Two Guided tracks, bounded normal handoff, native Loiter and observed RTL approach | Darwin ARM64, ArduPlane 4.6.3, plane |
| plane_takeover | RC takeover and sustained FBWA, rejection of a held Guided point outside Guided, explicit reacquisition | Darwin ARM64, ArduPlane 4.6.3, plane |
| sub | Surface-relative depth and eastbound motion, observed captured hold, native handback, bridge silence and Manual takeover | Darwin ARM64, ArduSub V4.6.0-dev, vectored |
| px4_offboard | Two measured stock-helm legs, silence to independent Hold, denied acquire under retained Offboard intent, native handback and landing | Linux ARM64, PX4 SIH 1.18.0-rc1, sihsim_quadx |

## Host cases

Build the selected checkout and the applicable targets against the pinned stock MOOS tree. All host cases use that one build. The native fixture holds the shared host lease during its run. Start the host cases serially with separate private output paths.

~~~sh
revision=$(git rev-parse HEAD)
cmake --build "$BUILD" --target iMAVLink mavmoos_telemetry mavmoos_parameters \
  mavmoos_ardurover_control rover_expanded_probe mavmoos_air_data mavmoos_arduplane_control \
  plane_native_probe mavmoos_plane_fault_tap mavmoos_ardusub_control ardusub_moos_probe

profile() {
  "$FIXTURE_PYTHON" -B scripts/release/profiles.py --revision "$revision" \
    --case "$1" --output "$RUN/$1" --build "$BUILD" --moos-root "$STOCK_MOOS" \
    --fixture-python "$FIXTURE_PYTHON" --rover-root "$ROVER_ROOT" \
    --plane-build-root "$PLANE_BUILD" --behavior-dir "$UAV_BASE/lib" --sub-root "$SUB_ROOT"
}
for case in rover_ground rover_boat plane_normal plane_takeover sub; do
  profile "$case" || exit
done
~~~

Select the existing qualified firmware artifacts and interpreter as described in the [Rover fixture](../../missions/rover_expanded/README.md), [Plane fixture](../plane_native/README.md), and [Sub fixture](../ardusub/README.md).

## Isolated Linux PX4 case

Use the source-built release work volume with the exact checkout under /work/mavmoos, build under /work/mavmoos-build, stock tools under /work/moos-ivp and altitude behavior under /work/altitude/build/lib. The build must include px4_offboard_moos_probe. Take the app and helm hashes from that build's receipt. Use the immutable local ARM64 tooling image and the existing read-only package volume containing the [pinned SIH binary and startup scripts](../px4_offboard/README.md).

~~~sh
"$FIXTURE_PYTHON" -B scripts/release/profiles.py --revision "$revision" \
  --case px4_offboard --output "$RUN/px4_offboard" \
  --px4-build-volume "$RELEASE_VOLUME" --px4-package-volume "$PX4_VOLUME" \
  --px4-app-sha256 "$LINUX_APP_SHA256" --px4-helm-sha256 "$LINUX_HELM_SHA256"
~~~

The launcher acquires the shared Linux-domain lease. The container uses a loopback-only network namespace, no added capabilities, a read-only root and read-only input mounts. The worker checks build source bytes before launching the fixture. The launcher removes only its exact owned container and leaves both build and firmware volumes available for the next qualification.

## Evidence and firmware candidates

Each output contains profile-result.json, launcher logs and the complete original native receipt under native/result.json. The normalizer requires observed scenario checks and verifies the original per-file SHA-256 index. A fresh output path preserves failures as separate attempts. Host binaries, case artifacts and results remain tied to their original run.

~~~sh
python3 -B -m unittest discover -s tests/release_profiles
~~~

The existing development pins remain the qualified baseline for Sub and PX4. The local ArduSub 4.5.7 tag resolves to 30257f01185471ab4c1ac544e47d1b4437e44c98 and needs a separate native build and reviewed candidate fixture inputs. The installed offline PX4 package covers the release-candidate pin. Stable versions require their own evidence rows after an equivalent owned run.
