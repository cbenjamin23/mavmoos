# 0028 — Passive observations and scoped payload components

November 6, 2026. Implemented in the local app and qualified at the scope recorded in the evidence ledger.

Keep optical flow, terrain, FC-reported navigation, link status, and clocks in optional passive providers. Extend the existing distance, odometry, and system-status providers for related observations. Producer-reported masks, unknowns and raw clocks remain explicit. These data do not infer physical response, control state, an atomic target, a fused position, or synchronized time.

Camera and gimbal have independent startup providers, each bound to one nominated component of the selected localhost FC system. The host allows only that provider's named observation messages and never treats an auxiliary heartbeat as vehicle identity. The primary, camera and gimbal duplicate histories have separate bounded storage. This prevents a payload flood from evicting a primary duplicate witness. Receive work still shares the app budget. There is no new sender, stream setup request, media retrieval, or control permission.

Use the pinned MAVROS paths and generated MAVLink definitions recorded in each provider guide. Use the pinned FC sender to establish what a firmware actually emits. Group the generated app tests by identity and the native evidence by simulator boot. A generated frame is evidence of application handling; it does not claim a native producer. Keep stock pHelmIvP and prior source/firmware receipts unchanged.
