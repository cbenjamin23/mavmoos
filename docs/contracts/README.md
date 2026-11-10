# Contract drafts

Current review package: [October 28, 2026, draft-01](2026-10-28-draft-01/README.md).

Follow-up: [prioritization and focused investigation](../research/2026-10-28-contract-priorities/README.md) ranks all 25 questions and develops recommendations for the first eight, especially coherent helm output and origin freshness. [Decision 0003](../decisions/0003-coherent-helm-and-origin-freshness.md) is Proposed. Draft-01 remains unchanged; the follow-up is research for review, not an adopted draft-02.

These documents propose MAVMOOS's observation, command-transaction, navigation-intent, and authority semantics. They are architecture drafts, not accepted decisions, implemented interfaces, or verified runtime behavior. MOOS variable names, serialization, library choices, and vehicle profiles remain subject to review.

Current constraint and sketch: [decision 0004](../decisions/0004-use-stock-moos-ivp.md) requires unmodified MOOS-IvP. The [built-in module package](../design/2026-10-29-built-in-modules/README.md) proposes a stock scalar/state compatibility path with explicitly weaker navigation coherence/origin claims. It is not silently certified against draft-01. A later contract revision must reflect that supported interface; the archived draft is unchanged.

The dated package is the archival unit. Put a substantive later revision in a new dated/draft-numbered directory, link its predecessor, and update this index. Preserve prior packages so changes in assumptions and contracts remain reviewable. Editorial corrections can be recorded in the relevant package's revision notes.
