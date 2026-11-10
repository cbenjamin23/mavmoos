"""Test-only receive fault and direct-wire audit for owned SITL routes."""

import json
from pathlib import Path

from plane_faults import fault_tap
from pymavlink.dialects.v20 import ardupilotmega as mav


FAULTS = ('ack', 'receive', 'restart')


def need(condition, message):
    if not condition:
        raise RuntimeError(message)


class Route:
    def __init__(self, out, library, ports, result):
        self.out, self.library, self.ports = out, library, ports
        self.data = result['confirmation_fault'] = {'traces': {}, 'times': {}}

    def environment(self, instance):
        self.control = self.out / f'{instance}.tap-mode'
        trace = self.out / f'{instance}.plf'
        fault_tap.set_mode(self.control, 'pass')
        self.data['traces'][instance] = trace.name
        return fault_tap.environment(self.library, self.ports['bridge'], self.ports['peer'], trace, self.control)

    def mark(self, name):
        self.data['times'][name] = fault_tap.steady_time()

    def drop(self, fault, command, mode=None, *, ack_only=False):
        self.data.update(kind=fault, command=command, mode=mode)
        ack_only = ack_only or fault == 'ack'
        self.data['ack_only'] = ack_only
        selective = {mav.MAV_CMD_NAV_TAKEOFF: 'drop_takeoff_ack',
                     mav.MAV_CMD_DO_SET_MODE: 'drop_mode_ack',
                     mav.MAV_CMD_NAV_RETURN_TO_LAUNCH: 'drop_rtl_ack'}[command]
        self.data['times']['fault'] = fault_tap.set_mode(self.control, selective if ack_only else 'drop_rx')[1]

    def discarded_ack(self):
        path = self.out / self.data['traces']['original']
        try:
            records = fault_tap.read_trace(path, self.ports['bridge'], self.ports['peer'])
        except ValueError as error:
            # The trace is still being written. A later poll reads the full record.
            if str(error).startswith('truncated'):
                return None
            raise
        for row in records:
            if row.disposition != 'i' or row.at < self.data['times']['fault']:
                continue
            for msg in mav.MAVLink(None).parse_buffer(row.data) or []:
                if (msg.get_type() == 'COMMAND_ACK' and
                        (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1) and
                        (msg.target_system, msg.target_component) == (245, 191) and
                        (msg.command, msg.result) == (self.data['command'], mav.MAV_RESULT_ACCEPTED)):
                    return row.at
        return None

    def restore(self):
        self.data['times']['restore'], _ = fault_tap.set_mode(self.control, 'pass')

    def exit_before_takeoff(self):
        self.data.update(kind='pre_restart', command=mav.MAV_CMD_NAV_TAKEOFF, mode=None)
        self.data['times']['fault'] = fault_tap.set_mode(self.control, 'exit_before_takeoff')[1]


def frames(out, result):
    ports = result['ports']
    fault = result['confirmation_fault']
    for instance, name in fault['traces'].items():
        records = fault_tap.read_trace(out / name, ports['bridge'], ports['peer'])
        startup = b''
        gyro_banner = False
        for row in records:
            need(row.disposition in ('T', 'R', 'i') or
                 (row.disposition == 'h' and fault['kind'] == 'pre_restart'), 'test changed outbound delivery')
            # Copter's original serial0 startup can include a tail of "**\n"
            # after peer discovery. Exclude only that bounded pre-fault prefix.
            if (result['profile'] in ('arducopter-4.6.3-quad', 'arducopter-4.6.3-hexa', 'arducopter-4.6.3-octa') and instance == 'original' and
                    row.disposition == 'R' and row.at < min(records[0].at + 2, fault['times']['fault'])):
                if row.data == b'Init Gyro' and not gyro_banner and not startup:
                    gyro_banner = True
                    continue
                if row.data in (b'*', b'\n'):
                    startup += row.data
                    need(b'**\n'.startswith(startup) or b'**\n'.endswith(startup), 'unexpected Copter startup bytes')
                    continue
            messages = mav.MAVLink(None).parse_buffer(row.data) or []
            need(messages and sum(len(msg.get_msgbuf()) for msg in messages) == len(row.data),
                 'incomplete direct-route MAVLink datagram')
            for msg in messages:
                identity = msg.get_srcSystem(), msg.get_srcComponent()
                if row.disposition in ('T', 'h'):
                    need(len(messages) == 1 and identity == (245, 191), 'unexpected bridge sender')
                elif identity != (1, 1):
                    continue
                yield instance, row.at, row.disposition, msg


def effectful(msg):
    if msg.get_type() == 'HEARTBEAT':
        return False
    if msg.get_type() == 'PARAM_REQUEST_READ' and (msg.target_system, msg.target_component) == (1, 1):
        return False
    if (msg.get_type() == 'COMMAND_LONG' and msg.command == mav.MAV_CMD_REQUEST_MESSAGE and
            (msg.target_system, msg.target_component, msg.confirmation, msg.param1) ==
            (1, 1, 0, mav.MAVLINK_MSG_ID_AUTOPILOT_VERSION) and
            not any(getattr(msg, f'param{index}') for index in range(2, 8))):
        return False
    return not (msg.get_type() == 'COMMAND_LONG' and msg.command == mav.MAV_CMD_SET_MESSAGE_INTERVAL and
                (msg.target_system, msg.target_component, msg.confirmation) == (1, 1, 0))


def audit(out, result):
    fault = result['confirmation_fault']
    times = fault['times']
    rows = list(frames(out, result))
    tested = [(at, msg) for _, at, side, msg in rows if side == 'T' and times['fault'] <= at < times['handback'] and
              effectful(msg)]
    need(len(tested) == 1, f'expected one effectful send through fault and retries, got {[(at,m.get_type()) for at,m in tested]}')
    sent, packet = tested[0]
    need(packet.get_type() == 'COMMAND_LONG' and packet.command == fault['command'] and
         (packet.target_system, packet.target_component, packet.confirmation) == (1, 1, 0) and
         (fault['mode'] is None or packet.param2 == fault['mode']), 'wrong tested command reached the wire')
    end = times['stopped'] if fault['kind'] == 'restart' else times['restore']
    incoming = [(at, side, msg) for instance, at, side, msg in rows if instance == 'original' and
                sent <= at < end and side != 'T']
    ack = [(at, side, msg) for at, side, msg in incoming if msg.get_type() == 'COMMAND_ACK' and
           msg.command == packet.command and (msg.target_system, msg.target_component) == (245, 191)]
    need(len(ack) == 1 and ack[0][1] == 'i' and ack[0][2].result == mav.MAV_RESULT_ACCEPTED,
         'accepted addressed ACK was not observed and discarded once')
    if fault.get('ack_only', fault['kind'] == 'ack'):
        need([(at, msg.get_type()) for _, at, side, msg in rows if side == 'i'] == [(ack[0][0], 'COMMAND_ACK')] and
             any(side == 'R' and msg.get_type() == 'HEARTBEAT' for _, side, msg in incoming),
             'selective ACK loss changed another packet or failed to pass heartbeat evidence')
        if fault['kind'] == 'restart':
            need(not any(side == 'R' and msg.get_type() == 'EXTENDED_SYS_STATE' and msg.landed_state == 2
                         for _, side, msg in incoming), 'original bridge received the in-air confirmation before restart')
    else:
        need(not any(side == 'R' for _, side, _ in incoming), 'original process received post-send observations')
    handback = [msg for _, at, side, msg in rows if side == 'T' and times['handback'] <= at < times['retry_after'] and
                effectful(msg)]
    wanted_stop = fault.get('handback_stop_mode')
    if wanted_stop is not None:
        need(len(handback) == 1 and handback[0].get_type() == 'COMMAND_LONG' and
             handback[0].command == mav.MAV_CMD_DO_SET_MODE and handback[0].param2 == wanted_stop,
             'explicit release did not issue exactly its one stop-only command')
    else:
        need(not handback, 'handback emitted an unexpected effectful packet')
    tail = [msg for _, at, side, msg in rows if side == 'T' and times['retry_after'] <= at < times['cleanup'] and
            effectful(msg)]
    need(not tail, 'a stale retry after handback reached the wire')
    need(any(at >= times['cleanup'] - 1 and side == 'R' for _, at, side, _ in rows), 'capture ended before final retry window')
    count = sum(1 for _, at, side, msg in rows if side == 'T' and times['fault'] <= at and
                msg.get_type() == 'COMMAND_LONG' and msg.command == fault['command'] and
                (fault['mode'] is None or msg.param2 == fault['mode']))
    return {'frames': len(rows), 'target_sends_through_retries': len(tested), 'target_sends_including_cleanup': count,
            'discarded_accepted_ack': True, 'handback_stop_sends': len(handback), 'post_handback_retry_sends': len(tail),
            'processes': list(fault['traces'])}


def replay(path):
    import hashlib

    receipt = json.loads(path.read_text())
    need(receipt['result'] == 'PASS', 'replay requires a passing native receipt')
    for name, expected in receipt['files'].items():
        digest = hashlib.sha256((path.parent / name).read_bytes()).hexdigest()
        need(digest == (expected['sha256'] if isinstance(expected, dict) else expected), f'evidence changed: {name}')
    if receipt['scenario'] == 'native-confirmation-loss':
        from rover_mission.native_confirmation import audit_native
        value = audit_native(path.parent, receipt)
    else:
        value = audit(path.parent, receipt)
    need(value == receipt['wire_audit'], 'wire readback differs')
    return value


if __name__ == '__main__':
    import sys

    print(json.dumps(replay(Path(sys.argv[1])), indent=2))
