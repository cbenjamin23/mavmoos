# Mode catalog

Select `plugins = telemetry,mode_catalog` for passive Standard Modes reports from the selected ArduPilot or PX4 flight controller on route 1/1. The provider observes `AVAILABLE_MODES` (435), `CURRENT_MODE` (436), `AVAILABLE_MODES_MONITOR` (437), and the primary heartbeat. It has no MOOS subscriptions or requests and grants no mode or helm authority. A peer that only streams its current mode leaves the available catalog empty. [MAVLink Standard Modes Protocol](https://mavlink.io/en/services/standard_modes.html)

| MOOS variable | What it reports |
| --- | --- |
| `MAVMOOS_MODE_CATALOG` | Advertised count and rows, coverage, conflicting records, current sequence and separately aged monitor evidence |
| `MAVMOOS_CURRENT_MODE` | Active custom and standard mode, intended custom mode when provided, display labels and exact observation source |

The table accepts at most 64 indexed rows per bounded collection window. It orders rows by index and marks coverage complete only when every distinct index from 1 through the advertised count is present. An observed newer 8-bit revision clears prior rows. Repeated revisions and identical rows leave the collection deadline at the first accepted entry. A conflicting row or count makes the collection incomplete until the current window expires or the sequence advances. Sequence ordering uses an 8-bit forward half range and recognizes wraparound. An unversioned list can have full index coverage with `sequence_bound=false`. The protocol cannot prove that delayed untagged rows belong to a common firmware revision.

Current and intended mode remain distinct. A newer heartbeat that disagrees with the active custom mode replaces the active view and clears the retained intended field until fresh current-mode evidence arrives. If current-mode reports stop, the provider uses the fresh heartbeat's numeric custom mode when its custom-mode flag is set. Labels come from an unambiguous match in a complete live catalog or fall back to numeric IDs. Intended custom mode zero means the sender did not supply the intent. Source, age, and session metadata accompany each observation.

Configure `plugin.mode_catalog.sample_timeout` in seconds, default 5, and `plugin.mode_catalog.publish_interval`, default 0.1. The collection, active report, and monitor each retain their own timestamp. A new heartbeat epoch clears the table and observed modes.

The pinned PX4 simulation returned 27 rows. The actual app matched those rows on exact replay and modeled checks cover sequence changes and incomplete lists. Mode evidence
