# Release evidence

The [latest release report](../implementation/2026-11-10-release-28/README.md) records the simulator results and the exact source revision tested before the public snapshot. The [control coverage matrix](control-coverage.md) maps supported commands and failure cases to tests. The small evidence bundles referenced by the capability registry contain the original records needed by the software checks.

The original Git history, detailed logs, and replay bundles are in the private MAVMOOS archive. The [archive index](evidence-index.tsv) identifies older ZIPs by path and SHA-256. With archive access, clone that repository and follow its evidence recovery guide to replay the historical runs. New software and simulator runs can be performed from this checkout using the [release campaign guide](sitl-release.md).
