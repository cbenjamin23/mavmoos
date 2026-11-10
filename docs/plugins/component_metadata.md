# Component metadata

The `component_metadata` provider binds a MAVLink 2 `COMPONENT_METADATA` declaration to one selected component on an owned loopback route. It verifies the declaration against a private local mirror supplied by the SITL fixture or operator. The checked document follows a defined subset of MAVLink General metadata version 1. [Component Metadata protocol](https://mavlink.io/en/services/component_metadata.html) · [General metadata schema](https://github.com/mavlink/mavlink/blob/master/component_metadata/general.schema.json)

```text
plugins = telemetry,component_metadata
component_metadata_component = 196
plugin.component_metadata.mirror_directory = /absolute/private/fixture
plugin.component_metadata.sample_timeout = 3
```

The mirror directory must already exist, belong to the current user, and have no group or other permissions or symbolic-link components. Put a private regular file named `general.json` inside it. The declaration URI must be `mftp:///component_metadata/general.json`. It selects this one local fixture path. Supply `version` 1 and an empty `metadataTypes` array. The optional vendor and model strings support bounded printable ASCII. Firmware and hardware versions use three or four numeric parts. The mirror's maximum document size is 4,096 bytes.

`MAVMOOS_COMPONENT_METADATA` reports the accepted source, session, original receipt age, declared CRC, verification outcome and parsed fields. Verification uses MAVLink's file CRC32 and retains a snapshot from the verification instant. A new declaration supersedes the previous verification. Duplicate, conflicting and regressed boot stamps preserve the age of the last accepted declaration. A conflict, regressed stamp, stale declaration or lost source clears the snapshot. Verification rejects a checksum mismatch, unsupported schema or URI, or a replaced or unsafe mirror file. The result carries observation scope. [MAVLink file CRC](https://mavlink.io/en/guide/crc.html)

This version verifies bytes from the explicitly mounted mirror. MAVFTP retrieval and linked metadata documents belong to separate transport work.
