# 0014: Paced parameter inventory beside flight control

Date: November 3, 2026. Status: implemented engineering choice under the authorized software/simulation continuation. Stock pHelmIvP and freshness limits are unchanged. No hardware qualification follows.

## Evidence and choice

Full `PARAM_REQUEST_LIST` streams interfered with primary-channel position delivery during two concurrent native Copter tests. One intervening higher-rate run passed, then the exact-build repeat failed again. Local receive budgets, queue-age bounds and independent FC telemetry did not show the same lapse. The pinned FC source slows most deferred streams during a queued parameter list and permits an additional selected-bucket delay. That is a source-supported interaction; the exact runtime scheduler bucket was not instrumented. Preserve both failures and the single candidate pass in the concurrent archive.

Do not hide this by extending navigation freshness or keep escalating stream rates. The default parameter inventory now issues standard indexed `PARAM_REQUEST_READ` messages, beginning at index zero to discover the count. It never starts the FC's full-list streaming state. This is standard MAVLink, with module-owned request accounting; UDP is unchanged. The tradeoff is an inventory measured in tens of seconds rather than a short burst.

## Bounds and receipt semantics

- At most four logical indices are outstanding, and at most one new parameter request enters the host per iteration. Control work retains priority.
- One original list deadline, default 60 seconds, is derived from the MOOS publication. Each index phase has at most one second to send and one second after verified local acceptance to reply, bounded by that original deadline. Retries never renew the operation.
- At most 32 recovery requests and 4,096 + 32 total requests are allowed. Local admission/I/O failure terminates explicitly. A late failed send cannot become a hidden retry.
- Only a requested concrete index, after its matching local-send callback and inside that phase's response window, can advance completeness. Count/name/type/value checks, 4,096-row and 1 MiB snapshot limits remain. Unindexed change notifications cannot advance the list.
- `PARAM_VALUE` supplies no request nonce. These checks establish a post-send observation of a requested index, not causal correlation or an atomic FC database snapshot. A read already queued at local termination can remain until its bounded send deadline; no remote recall is promised.

At 50 Hz, 1,364 requests require at least about 27 seconds; at 30 Hz about 46 seconds. The 60-second deadline is a declared profile change, not a claim that every table or link can complete. A bounded `INCOMPLETE` or session cancellation must remain distinct from successful inventory and continuous navigation.

## Legacy streamed bench

`parameter_list_mode=streamed` remains an explicit disarmed, fresh-ON_GROUND bench option, with its original default 20-second list deadline. The app rejects this mode when command/helm/other feature modules are loaded; the host independently checks the loaded feature names, observed ground state and absence of authority/commands before admission and final send. Only telemetry/parameters may compose that bench. This prevents our own command feature from arming after list admission. External GCS/pilot changes are not atomically controlled.

Named reads and default paced inventory remain independent of navigation permission and armed/airborne state. Restricted simulator SET retains its separate ground/write/command-transition policy. General writes, FTP/extended parameter transport and concurrent multi-client transaction isolation remain future work.

## Reference transfer and validation

MAVROS informed list accounting, indexed recovery and per-feature timers; pArduBridge supplied useful independent named reads. The choice of paced inventory follows the measured interaction with this pinned Copter, rather than copying a reference's exact list policy. See the parameter interface, scheduler evidence, FC source diagnosis and execution ledger. Fake-clock scheduling equality, native flight results and Linux builds are separate evidence layers.
