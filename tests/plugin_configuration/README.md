# Checked startup configuration review

This suite loads adversarial shared libraries through the real `PluginHost` and
`Runtime`, with an inert clock and fake sender. It exercises callbacks, selected
identity, complete owner batches, structural limits, provider replacement and
every current state-changing `Services` method at factory, checked, legacy and
discard boundaries. Startup diagnostic publications and read-only queries are
allowed. Callback attempts that catch their own rejection still invalidate the
runtime. Separate deferred-startup controls demonstrate the final subscription
getter cannot acquire shadow authority, the attempt survives another startup
callback, and clean input/dispatch starts only after explicit finish. No socket,
MOOS process, simulator or vehicle is used.

The fixture's own audit exports are a test-only inspection surface. They do not
extend the SDK. Its deliberately ABI9 factory proves rejection before instance
construction; an actual separately built frozen ABI8 example was tested in the
independent consumer walkthrough, with platform and artifact identity recorded.

The parent build can add this directory and run `ctest -R '^plugin_configuration$'`.
A standalone build compiles the matching checkout's actual runtime sources:

```sh
cmake -S tests/plugin_configuration -B build-sdk-configuration \
  -DMAVMOOS_SOURCE_ROOT="$PWD" \
  -DMAVMOOS_MAVLINK_ARCHIVE=/absolute/path/mavlink-a2d0ecf.tar.gz
cmake --build build-sdk-configuration --parallel 4
ctest --test-dir build-sdk-configuration --output-on-failure
```

Use a fresh second directory with `-DMAVMOOS_CONFIGURATION_SANITIZERS=ON` for
ASan/UBSan. Successful component tests do not establish the actual app path,
documented mission, independent zero-output capture, or whole Linux/SITL
qualification. Those remain separate required rows in the completion review.
