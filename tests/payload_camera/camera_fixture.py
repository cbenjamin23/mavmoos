"""A selected MAVLink camera on a private simulated route."""

import math
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink


class CameraFixture:
    def __init__(self, codec, directory, component=100):
        if (codec.srcSystem, codec.srcComponent) != (1, component):
            raise ValueError("camera encoder must use the selected fixture identity")
        self.codec = codec
        self.component = component
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.captured_paths = []
        self.accepted_sequences = []
        self._last_status_ms = None
        self._pending_at = None
        # A real MAVLink 2 zero count can trim the entire count extension.
        # A seed image gives the first request an independently known count.
        self._store_image()

    @property
    def capture_count(self):
        return len(self.captured_paths)

    def _store_image(self):
        index = self.capture_count
        path = self.directory / f"capture-{index:05d}.pgm"
        pixels = bytes((index * 29 + i) % 256 for i in range(64))
        with path.open("xb") as image:
            image.write(b"P5\n8 8\n255\n" + pixels)
        self.captured_paths.append(path)
        return index

    def periodic(self, boot_ms):
        messages = []
        if self._pending_at is not None and boot_ms >= self._pending_at:
            index = self._store_image()
            self._pending_at = None
            messages.append(self.codec.camera_image_captured_encode(
                boot_ms, 0, 0, 0, 0, 0, 0, (1.0, 0.0, 0.0, 0.0), index, 1, b""
            ))
            self._last_status_ms = None
        if self._last_status_ms is None or boot_ms - self._last_status_ms >= 300:
            self._last_status_ms = boot_ms
            messages.extend((
                self.codec.heartbeat_encode(mavlink.MAV_TYPE_CAMERA, mavlink.MAV_AUTOPILOT_INVALID,
                                            0, 0, mavlink.MAV_STATE_ACTIVE),
                self.codec.camera_information_encode(
                    boot_ms, list(b"MAVMOOS".ljust(32, b"\0")), list(b"SITL camera".ljust(32, b"\0")),
                    0, 1.0, 1.0, 1.0, 8, 8, 0, mavlink.CAMERA_CAP_FLAGS_CAPTURE_IMAGE, 0, b""
                ),
                self.codec.camera_capture_status_encode(
                    boot_ms, int(self._pending_at is not None), 0, 0.0, 0, 100.0, self.capture_count
                ),
            ))
        return messages

    def handle(self, frame, boot_ms):
        if frame.get_type() != "COMMAND_LONG" or frame.command != mavlink.MAV_CMD_IMAGE_START_CAPTURE:
            return []
        if (frame.target_system, frame.target_component) != (1, self.component):
            return []
        if (frame.get_srcSystem(), frame.get_srcComponent()) != (245, 191):
            return []
        sequence = frame.param4
        valid = (frame.confirmation == 0 and
                 (frame.param1, frame.param2, frame.param3, frame.param5, frame.param6) == (0, 0, 1, 0, 0) and
                 math.isnan(frame.param7) and
                 math.isfinite(sequence) and sequence == int(sequence) and 1 <= sequence < 1 << 24)
        if not valid:
            result = mavlink.MAV_RESULT_DENIED
        elif self._pending_at is not None or (self.accepted_sequences and sequence <= self.accepted_sequences[-1]):
            result = mavlink.MAV_RESULT_TEMPORARILY_REJECTED
        else:
            self.accepted_sequences.append(int(sequence))
            self._pending_at = boot_ms + 150
            result = mavlink.MAV_RESULT_ACCEPTED
        return [self.codec.command_ack_encode(frame.command, result, 0, 0, 245, 191)]
