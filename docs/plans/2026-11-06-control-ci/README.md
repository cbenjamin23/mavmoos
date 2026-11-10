# Control commands and transitions in local CI

The work covers the implemented MOOS control interfaces for Copter, ground Rover, boat, conventional Plane, vectored Sub and the PX4 quad. Read the dispatchers and existing tests together. Record which command forms, mode transitions and failure paths have a required software check, and which have independent simulator evidence.

Treat command acknowledgement, observed mode, observed motion and a retired authority grant as separate outcomes. Use deterministic tests for combinations of ownership, stale or conflicting inputs, command replies and cancellation. Use owned simulator scenarios for the aircraft and boat responses those tests cannot establish. Preserve the firmware, platform and artifact scope on each native result.

The acceptance criteria are a reviewable command and transition inventory, newly asserted failure or transition behavior where the audit finds gaps, required local CI selection for those checks, and an archived result from the app's normal test and simulator paths. Use the existing runner and evidence format. This work uses stock pHelmIvP and the existing owned simulation boundaries.

The [control guide](../../development/control-qualification.md) and qualification ledger record the delivered coverage and exact results.
