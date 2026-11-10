MAVMOOS for Linux ARM64
=======================

Extract the archive wherever the account that runs MOOS can read it. The kit contains `bin/iMAVLink`, the `.so` modules listed in its manifest, the matching MOOSGeodesy library and notices, and a receive-only mission example. Use Linux AArch64 with a compatible C++ runtime and stock MOOS-IvP tools on `PATH`. The release witness uses the pinned Ubuntu 24.04 Linux toolchain and stock MOOSDB and pAntler. Its package receipt records the actual image and artifact digests.

From the extracted `mavmoos` directory, verify the payload and display the bridge help.

```sh
sha256sum --check SHA256SUMS
./bin/iMAVLink --help
```

Copy `share/mavmoos/receive-only.moos.in` into a private mission directory. Replace `@PLUGIN_DIRECTORY@` with the absolute path to this kit's `plugins` directory. Pick unused loopback ports and configure the fixture peer for the same pair. With stock MOOS tools on `PATH`, add this kit's `bin` first and run `pAntler receive-only.moos` from that directory. This example observes MAVLink supplied by an owned local simulator or replay fixture.
