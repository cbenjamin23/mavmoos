# Evidence, provenance, and limits

Collected October 28, 2026. Static source/documentation inspection and architectural reasoning only. The 14-file source manifest records exact revisions, immutable URLs, Git blob IDs, SHA-256 hashes, byte/line counts and acquisition method, plus one separately hash-identified local artifact. No build, mission, simulator, FC connection or vehicle command was executed.

## Reproducibility and scope

- **MOOS-IvP:** local `~/moos-ivp`, revision `023fb54e00692611ecee2cd6f1138d10eec2127e`. The nine selected tracked files matched that revision byte for byte. Other local changes were preserved.
- **Core MOOS:** `themoos/core-moos` revision `24942423ce3f3ebdb3cee876edb71bdd04dc5555`. The local database, app, communications-client and message files matched the independently fetched/upstream-identified Git blobs. These third-party bytes are not attributed to the outer MOOS-IvP commit.
- **AppCasting time provenance:** the local third-party file differs from that Core MOOS revision. Its relevant assignment was inspected separately and its full-file hash recorded. The upstream pinned comparison was also fetched; it is not represented as the same file. See E6.
- **Earlier bridges/libraries:** the previous source manifest remains the baseline for the pArduBridge, MAVSDK, MAVROS and BlueBoat comparisons. No updated remote branch state is assumed.
- **Documentation:** the live MIT PDF index was retrieved again before selecting the uHelmScope manual. The web reader could not access the index; direct HTTPS retrieval succeeded. Existing archived chapters were reused. PDFs have access-date/hash provenance, not an assumed match to a source release.

Line links below refer to the named immutable source revisions. Manual line spans are coordinates in the repository's numbered `pdftotext -layout` extractions, not PDF page anchors. Source inspection settles checkout-specific order/format details that the manuals do not fully specify.

## E1 Helm cycle and output order

- [HelmIvP.cpp 340–422](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L340-L422): state post, early returns, manual override, startup delay, solver call.
- [HelmIvP.cpp 437–549](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L437-L549): behavior posts, journal summary, final policy/completeness checks and per-domain scalar output.
- [HelmIvP.cpp 577–602](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L577-L602): iteration notification is inside `postBehaviorMessages`, before the later checks/scalars.
- [HelmEngine.cpp 91–121](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmEngine.cpp#L91-L121) and [428–464](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmEngine.cpp#L428-L464): increment/clear/solve sequence and final decision accumulation.

**Finding:** iteration and summary are after solver return but before the helm's final output checks. Neither is a direct “all final scalars committed” marker. **Inference:** a complete post-policy envelope would remove the need to infer that boundary from separate variables.

## E2 Summary encoding and parsing

- [HelmReport.cpp 187–203](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/lib_helmivp/HelmReport.cpp#L187-L203): decision group enumerates domain variables and writes `varbalk` for absent decisions.
- [HelmReport.cpp 574–611](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/lib_helmivp/HelmReport.cpp#L574-L611) and [668–680](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/lib_helmivp/HelmReport.cpp#L668-L680): always-present iteration/time, change-dependent fields, decision group, halted/active-goal and domain/halt fields.
- [HelmReportUtils.cpp 65–118](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/lib_helmivp/HelmReportUtils.cpp#L65-L118) and [119–206](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/lib_helmivp/HelmReportUtils.cpp#L119-L206): copies prior report, clears decisions only upon a new decision group, parses numeric tokens with `atof`, and has no branch restoring `active_goal`.

**Findings:** journal reconstruction needs prior state; numeric parsing in this display utility does not preserve the missing-decision token; serializer and parser are not a lossless control codec. **Inference:** a control consumer requires its own strictly specified parsing/continuity contract if this source is used.

## E3 Restart and rejournal

- [HelmIvP.cpp 137–167](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L137-L167): restart preparation deletes the helm engine and clears output state.
- [HelmIvP.cpp 286–299](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L286-L299): restart handling and Boolean rejournal request.
- [HelmIvP.cpp 461–469](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L461-L469): full report on request, then deltas. [HelmEngine.cpp 44–50](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmEngine.cpp#L44-L50) initializes engine iteration to zero.

**Findings:** engine identity can change within one process; a returned report does not echo a correlated challenge token. **Inference:** app name/PID/iteration alone should not be the canonical producer incarnation or proof of a particular receiver's freshness exchange.

## E4 Summary numeric representation

- [MBUtils.h 94–97](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/lib_mbutil/MBUtils.h#L94-L97): five-decimal default for the decision formatter.
- [MBUtils.cpp 914–946](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/lib_mbutil/MBUtils.cpp#L914-L946): formatted representation and compacting.

**Finding:** the journal's comparison is of formatted decision-summary text, not exact numeric equality with the scalar doubles. Quantization suitability is a profile question, not a measured error claim for all values.

## E5 Core MOOS messages, mail, and retention

- [MOOSMsg.cpp 63–82](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/Comms/MOOSMsg.cpp#L63-L82): default notification construction assigns `MOOSTime()` unless a timestamp is supplied. [505–521](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/Comms/MOOSMsg.cpp#L505-L521) compares absolute clock skew to a tolerance, rather than establishing a calibrated bound on origin age.
- [MOOSCommClient.cpp 1019–1074](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/Comms/MOOSCommClient.cpp#L1019-L1074): source-name assignment/preservation options, message IDs, finite outbox and overflow removal. [1383–1427](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/Comms/MOOSCommClient.cpp#L1383-L1427) constructs individual notifications and optional source-auxiliary metadata.
- [MOOSApp.cpp 138–147](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/App/MOOSApp.cpp#L138-L147), [541–548](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/App/MOOSApp.cpp#L541-L548), and [709–721](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/App/MOOSApp.cpp#L709-L721): configurable mail-time sorting, enabled by default, before app mail handling.
- [MOOSDB.cpp 741–761](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/DB/MOOSDB.cpp#L741-L761), [811–830](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/DB/MOOSDB.cpp#L811-L830), and [925–968](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/DB/MOOSDB.cpp#L925-L968): original versus database-write times, metadata, notification-period filtering, and retained value delivery on registration.

**Findings:** transport metadata and queues do not themselves establish a helm generation, original-producer age or authenticated identity. This is not a claim that MOOS can only deliver one value per key; mail can contain multiple notifications. Nor is it a deployment-wide security audit.

## E6 Cycle time provenance

[HelmEngine.cpp 91–98](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmEngine.cpp#L91-L98) captures the passed time, and [140–144](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmEngine.cpp#L140-L144) puts it in the report before behavior production/solving. The local third-party `AppCastingMOOSApp.cpp`, lines 72–75, increments app iteration and assigns `m_curr_time` from `MOOSTime()` at the start of its Iterate method, which `HelmIvP::Iterate()` calls first.

That local file is `~/moos-ivp/MOOS/MOOSCore/Core/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.cpp`, SHA-256 `4138d9b5a3d9d98c53430afcd142547c01ee5411c4a58221c181577ff7d2de74`. Its revision was not independently identified. The [pinned upstream comparison at lines 59–62](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.cpp#L59-L62) has the same relevant assignment, but different full-file bytes. Both artifacts are distinguished in the manifest.

**Finding:** the report time in the inspected path is cycle-entry context, not a separately sampled post-solve completion clock. The individual scalar notifications use their normal notification-time path. **Inference:** qualifying origin freshness requires explicit event provenance and a clock/window bound; a post-solve publication stamp alone hides computation age. Source inspection does not identify the exact binary the user has run or measure its clock error.

## E7 All-stop and standby

- [HelmIvP.cpp 1717–1769](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L1717-L1769): repeated-reason suppression, initial incomplete/no-goal grace, scalar zeros without normal prefix.
- [HelmIvP.cpp 254–280](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L254-L280): source-based coordination, delayed primary handling and disable path. [1048–1064](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L1048-L1064) posts state and the promoted-standby suffix; [1211–1221](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L1211-L1221) checks standby heartbeat timeout.

**Finding:** native helm coordination and all-stop are not bridge authority epochs. **Recommendation:** preserve their evidence while applying explicit bridge-owned admission/transfer policy.

## M1 MIT manuals

| Primary document | Archived extraction / supporting lines |
| --- | --- |
| [Helm as a MOOS application](https://oceanai.mit.edu/ivpman/pdfs/chap_helm_as_moos.pdf) | Numbered text: 140–175 all-stop; 459–479 journal/state; 585–623 delayed primary/standby and assumed tolerated disturbance |
| [uHelmScope](https://oceanai.mit.edu/ivpman/pdfs/app_uhelmscope.pdf) | Numbered text: 118–125 missing optional decision (`varbalk`); 299–324 state/summary and full-then-journal operation |
| [MOOS chapter](https://oceanai.mit.edu/ivpman/pdfs/chap_moos.pdf) | Numbered text: 140–207 registration/mail/scheduling; 235–238 tick settings |

Manual archive/manifest records source URLs, access date, byte counts and original/extraction hashes. The new uHelmScope PDF is archived without edits. The index shortlist for this pass was the three filenames above; source inspection resolves implementation order, parser behavior and time provenance beyond the manuals.

## P1 Prior bridge and library evidence

The [previous continuous-setpoint trace](../2026-10-28-operating-models/trace-03-continuous-setpoints.md) and evidence index document pArduBridge's per-expression locking and receipt-age model, sender scheduling, MAVROS callback-driven setpoints, transport queues, and the BlueBoat cached-forwarding scenario. The [command trace](../2026-10-28-operating-models/trace-02-acknowledged-command.md) documents command helper ACK/retry/result behavior and separate state confirmation. Their original versioned links/hashes remain authoritative for those comparisons.

No additional FC-internal execution or physical-identity source trace was performed here. Window bounds, trust assumptions, replay/recovery rules and profile recommendations are new architectural reasoning, not discovered features of those existing bridges. The [paper cases](review-cases.md) are proposed review/validation obligations; none are executed test evidence.
