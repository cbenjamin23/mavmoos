# Design references

- [MAVROS source at the revision used for the original architecture study](https://github.com/mavlink/mavros/tree/2d3f4db1101c7950c8817a9d788f0d11106fb261) covers transport, vehicle context and plugin boundaries. The [provider crosswalk](mavros-crosswalk.md) compares those capabilities with MAVMOOS.
- [MOOS-IvP manuals](https://oceanai.mit.edu/ivpman/) document MOOS communications, the IvP Helm and behavior configuration. [pArduBridge](https://github.com/cbenjamin23/moos-ivp-uav-base) informed the MOOS vehicle workflows.
- The [MAVLink guide](https://mavlink.io/en/) describes the protocol. [ArduPilot developer documentation](https://ardupilot.org/dev/) and [PX4 documentation](https://docs.px4.io/) describe the autopilot interfaces and simulation environments.
