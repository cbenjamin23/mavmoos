# Broader Copter scenarios and capability registry

These are owned native simulation fixtures for the existing public Copter operation API and unchanged stock pHelmIvP. They add no vehicle command, mission execution, general parameter write, RC/servo injection, external range, physical device or arbitrary relay route. The standalone C++ target is an ordinary MOOS test client, not a replacement helm/app.

| Stable scenario ID | Scenario and acceptance |
| --- | --- |
| `native.copter.route_hold_return` | Fresh disarmed readiness → Guided/arm/takeoff 5 m → stock-helm point route → confirmed helm hold/cancel → second route → confirmed Guided hold/cancel → helm return home → native land/disarm |
| `native.copter.wind_repeated_cycle` | The same complete workflow twice in one app/simulator session, with distinct operation IDs and fixed preboot 2 m/s wind. Both cycles must finish disarmed/on ground; no in-air parameter write |

Each held-target window is 4 s. Acceptance checks late advancing original NAV sample stamps, stable XY/home-relative height, continuing accepted NAV throughout the window, exact original operation deadlines, cancellation outcomes and a quiet no-permission/no-new-NAV tail. A separate passive FC endpoint confirms late positions, low hold speed, armed Guided/in-air state and final ground/disarmed state. Closed DataFlash corroborates fixture parameters, allowed modes, ordinary arming and complete cycle count. Probe, Python receive and FC boot clocks remain separate; no source-age or physical accuracy guarantee is inferred.

The 300/520 s native supervisors cover GPS/EKF convergence plus one/two complete flight cycles and bounded landing. A timeout fails and cleans only owned process trees/selected ports. Source/build manifests and attestations are verified before sockets/processes and again after shutdown; exact executable/PID/profile/loopback proof remains in the production sender. Startup diagnostics are limited to the pinned finite SiK six-byte prefix before the first valid passive frame and retained separately. All output directories must be fresh.

The runner CLI is:

```sh
python3 -B tests/copter_scenarios/run_scenarios.py \
  --source-root /path/to/mavmoos --build /path/to/new-build \
  --ardupilot-root /path/to/verified-native-source --moos-root /path/to/moos-ivp \
  --behavior-dir /path/to/uav/lib --scenario route_hold_return \
  --firmware-manifest /path/to/profile.json --firmware-attestation /path/to/attestation.json \
  --expected-app-sha256 APP_SHA256 --expected-probe-sha256 PROBE_SHA256 \
  --output /path/to/fresh-result-directory
```

The root integrator grants serial native windows only after a matching artifact freeze. A prepared/compiled scenario is not a native pass. Root archives actual native execution separately.

`ScenarioEvidenceTests.py` supplies two self-contained synthetic positives and 32 malformed-evidence controls. `RegistryTests.py` validates strict schema, configured inventory, exact historical source/firmware/artifact/operation cells and negative controls. Both are offline Python checks; their counts are not additional native flights. Run them with `python3 -B tests/copter_scenarios/ScenarioEvidenceTests.py` and `python3 -B tests/copter_scenarios/RegistryTests.py --source-root .`.

`config/capabilities/copter.json` separates29 capability declarations from concrete qualifications and stable test IDs. Its historical 11 cells apply only to source c1c38e9 / app 0.8 / pinned 4.7.0-dev / Darwin arm64 / exact archived operation workflow. Newly declared scenarios remain pending until actual evidence exists. Root adds current qualification as additive evidence rather than rewriting frozen definitions during CI.

Root build wiring: `add_subdirectory(copter_scenarios)` in `tests/CMakeLists.txt`, plus Python CTests named `capability_registry` and `copter_scenario_audit`. Required new software IDs must be synchronized with the actual configured graph; missing required cases and configured-but-unregistered cases fail registry/CI validation.
