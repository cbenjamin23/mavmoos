"""A deterministic gimbal manager and device for the owned MAVLink payload bench."""

import json
import math
from pathlib import Path


class GimbalFixture:
    def __init__(self, codec, directory, component=154):
        if not 7 <= component <= 255:
            raise ValueError("fixture requires a component-native gimbal")
        self.codec = codec
        self.component = component
        self.primary = (0, 0)
        self.pitch_deg = self.yaw_deg = 0.
        self.target = (0., 0.)
        self.commands = []
        self._trace = Path(directory) / "gimbal_component_trace.jsonl"
        self._last_update = self._last_report = self._last_heartbeat = None

    def snapshot(self):
        return {
            "component": self.component,
            "primary_system": self.primary[0],
            "primary_component": self.primary[1],
            "pitch_deg": self.pitch_deg,
            "yaw_deg": self.yaw_deg,
            "target_pitch_deg": self.target[0],
            "target_yaw_deg": self.target[1],
            "commands": list(self.commands),
        }

    def _record(self, event, boot_ms, **fields):
        with self._trace.open("a", encoding="utf-8") as trace:
            trace.write(json.dumps({"event": event, "boot_ms": boot_ms, **fields}, allow_nan=False) + "\n")

    def _advance(self, boot_ms):
        if self._last_update is not None:
            if boot_ms < self._last_update:
                raise ValueError("fixture clock moved backwards")
            before = (self.pitch_deg, self.yaw_deg)
            step = 0.12 * (boot_ms - self._last_update)
            self.pitch_deg += max(-step, min(step, self.target[0] - self.pitch_deg))
            self.yaw_deg += max(-step, min(step, self.target[1] - self.yaw_deg))
            if before != self.target and (self.pitch_deg, self.yaw_deg) == self.target:
                self._record("target_reached", boot_ms, pitch_deg=self.pitch_deg, yaw_deg=self.yaw_deg)
        self._last_update = boot_ms

    def periodic(self, boot_ms):
        self._advance(boot_ms)
        if self._last_report is not None and boot_ms - self._last_report < 80:
            return []
        self._last_report = boot_ms
        codec = self.codec
        wire_ms = boot_ms & 0xFFFFFFFF
        packets = []
        if self._last_heartbeat is None or boot_ms - self._last_heartbeat >= 800:
            self._last_heartbeat = boot_ms
            packets.append(codec.MAVLink_heartbeat_message(
                codec.MAV_TYPE_GIMBAL, codec.MAV_AUTOPILOT_INVALID, 0, 0, codec.MAV_STATE_ACTIVE, 3))
            caps = (codec.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS | codec.GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW |
                    codec.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS | codec.GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW)
            packets.append(codec.MAVLink_gimbal_device_information_message(
                wire_ms, b"MAVMOOS", b"Simulated gimbal", b"", 1, 1, self.component, caps, 0,
                0., 0., math.radians(-75), math.radians(20), -math.pi / 2, math.pi / 2))
        packets.append(codec.MAVLink_gimbal_manager_status_message(
            wire_ms, codec.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME, self.component, *self.primary, 0, 0))
        pitch, yaw = math.radians(self.pitch_deg) / 2, math.radians(self.yaw_deg) / 2
        cp, sp, cy, sy = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        packets.append(codec.MAVLink_gimbal_device_attitude_status_message(
            0, 0, wire_ms, codec.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
            [cp * cy, -sp * sy, sp * cy, cp * sy], 0., 0., 0., 0))
        return packets

    def inject_takeover(self, boot_ms, system=42, component=190):
        self._advance(boot_ms)
        if not (1 <= system <= 255 and 1 <= component <= 255) or (system, component) == (245, 191):
            raise ValueError("takeover requires an independent controller")
        self.primary = (system, component)
        self.target = (self.pitch_deg, self.yaw_deg)
        self._record("takeover", boot_ms, system=system, component=component)

    def handle(self, frame, boot_ms):
        if frame.get_type() != "COMMAND_LONG" or (frame.target_system, frame.target_component) != (1, self.component):
            return []
        codec = self.codec
        configure = codec.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        point = codec.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        if frame.command not in (configure, point):
            return []
        self._advance(boot_ms)
        source = (frame.get_srcSystem(), frame.get_srcComponent())
        action = "rejected"
        if source == (245, 191) and frame.param7 == self.component and frame.confirmation == 0:
            if frame.command == configure:
                role = (frame.param1, frame.param2, frame.param3, frame.param4, frame.param5, frame.param6)
                if role == (-2, -2, -1, -1, 0, 0) and self.primary == (0, 0):
                    self.primary = source
                    action = "claim"
                elif role == (-3, -3, -1, -1, 0, 0) and self.primary == source:
                    self.primary = (0, 0)
                    action = "release"
            elif (self.primary == source and -75 <= frame.param1 <= 20 and -90 <= frame.param2 <= 90 and
                  math.isnan(frame.param3) and math.isnan(frame.param4) and
                  frame.param5 == codec.GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME and frame.param6 == 0):
                self.target = (frame.param1, frame.param2)
                action = "point"
        self.commands.append(action)
        self._record("command", boot_ms, action=action, command=frame.command, source=list(source),
                     primary=list(self.primary), target=list(self.target))
        result = codec.MAV_RESULT_DENIED if action == "rejected" else codec.MAV_RESULT_ACCEPTED
        return [codec.MAVLink_command_ack_message(frame.command, result, 0, 0, *source)]
