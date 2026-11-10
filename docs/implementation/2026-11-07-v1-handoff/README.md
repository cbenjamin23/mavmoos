# V1 handoff verification

The [from-scratch guide](../../development/from-scratch.md) starts with a shared pinned workspace. The workflow catalog selects the required software witnesses for all 36 providers and lists the named native cases with their inputs, outcomes and logs.

## Fresh source setup

On Apple Silicon macOS, the setup fetched the public stock MOOS-IvP, altitude behavior and ArduCopter 4.6.3 pins into an empty workspace. It built and checked the stock tools, the app and the simulator. The first attempts exposed an inherited compiler shim and a missing `uProcessWatch` target. Selecting the installed standalone command-line toolchain and adding the stock build target resolved both.

The receive-only check passed with zero egress. The owned Copter quad starter reached 5.023 m above home, ran both waypoint legs, held its final station, handled bounded handoff and pilot Loiter takeover, then landed and disarmed. The fixture independently read a parameter while airborne and verified process and port cleanup. The run used the public stock pin and a newly attested simulator build.

The first full software run passed 164 of 165 checks. The generated-peer Rover and boat check required the historical MOOS revision. That test now checks either of the two recorded stock revisions and verifies the selected source before and after its four receive-only and shadow cases. The full rerun passed all 165 checks. The exact native qualification fixtures use their original input requirements. Verified results

The public setup covers the software recipes and Copter starter. Several other native recipes still need the exact host binaries, simulator artifacts or Linux volumes named in the [release guide](../../development/sitl-release.md). The new starter result is scoped to this workspace and platform. The earlier full release evidence retains its own inputs and revision.

## Lean checkout and site

The working tree retired 157 indexed historical archives, 58 redundant ZIP staging files and four regenerable TAR exports. A shallow checkout retained about 252 MiB of tracked files and passed the registry evidence check. Restoring the four latest release archives from Git reproduced the 24-stage verdict byte for byte. [Archive retrieval](../../development/evidence.md)

The [static documentation site](../../../site/README.md) builds with Python and loads from a local server. The five pages use relative navigation. Browser checks covered workflow search, recipe expansion, command copying and the simplified page headings. The results page presents the recorded SITL outcomes and a source-level MAVROS comparison.
