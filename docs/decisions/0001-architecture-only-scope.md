# 0001: Establish an architecture-only repository

Date: October 28, 2026. Status: accepted scope.

## Context and authorization

The user requested: “set up the mavmoos repo with the README and docs you suggested, archive the notes, and keep it architecture-only for now”. The supplied notes likewise identify architecture analysis as the next activity.

## Decision

Initialize a standalone local Git repository named `mavmoos`, with a medium-length README, current architecture, annotated references, capability map, discussion archive, and decision records. Preserve the supplied attachment verbatim and document its provenance.

Keep the repository documentation-only. Do not introduce bridge source code, a build system, dependency installation, runnable missions, or implementation scaffolding during this phase. Remote publication is outside this setup action.

## Alternatives and consequences

An extension-repository scaffold could accelerate later implementation, but would introduce premature build and dependency choices. Keeping all discussion only in chat would make it harder to track changing conclusions and source revisions. A documentation repository preserves the research basis while leaving these choices open.

## Next review

Proceed with architecture analysis when requested, starting with the two MAVROS source traces in [architecture.md](../architecture.md#next-architecture-analysis). Move into implementation only after an explicit scope change. No additional architectural mechanism is accepted by this record.
