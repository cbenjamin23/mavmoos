#!/usr/bin/env python3
"""Optional actual pymavlink byte controls. Memory only; no transport/native I/O."""
import copy
import unittest
from pymavlink.dialects.v20 import ardupilotmega as mavlink
from wire_evidence import FrameDecoder,replay


def codec(system=1):
    result=mavlink.MAVLink(None);result.srcSystem=system;result.srcComponent=1;return result


def heartbeat(system=1,armed=False):
    output=codec(system);return bytes(output.heartbeat_encode(2,3,128 if armed else 0,0,3).pack(output))


def imu():
    output=codec();return bytes(output.raw_imu_encode(1234567,1000,-2000,0,1000,-2000,125,1000,-250,0,id=0,temperature=2350).pack(output))


class WireCodecTests(unittest.TestCase):
    def test_standard_frames_replay_fields_and_order_without_reconstructing_local_time(self):
        raw=heartbeat()+imu();parser=FrameDecoder(codec());rows,startup=parser.feed(raw,123)
        self.assertEqual(startup,[]);self.assertEqual(parser.finish()['frames'],2)
        report=replay(raw,rows,codec);self.assertTrue(report['same_message_fields_and_order']);self.assertFalse(report['local_read_timestamps_reconstructed'])
        self.assertEqual(rows[1]['xacc'],1000);self.assertEqual(rows[1]['temperature'],2350)

    def test_exact_six_byte_startup_prefix_is_retained_separately(self):
        raw=b'0 '*3+heartbeat()+imu();parser=FrameDecoder(codec());rows,startup=parser.feed(raw,10)
        self.assertEqual(parser.finish()['startup_bytes'],6);self.assertEqual(b''.join(bytes.fromhex(r['raw_hex']) for r in startup),b'0 '*3)
        self.assertEqual(replay(raw,rows,codec)['startup_bytes'],6)

    def test_truncated_terminal_mavlink_frame_is_not_silently_accepted(self):
        parser=FrameDecoder(codec());parser.feed(heartbeat()+imu()[:-1],1)
        with self.assertRaisesRegex(ValueError,'incomplete/unconsumed'):parser.finish()

    def test_later_noise_bad_checksum_or_partial_prefix_is_rejected(self):
        for raw in (heartbeat()+b'x',b'0 0 '+heartbeat(),b'x'+heartbeat(),heartbeat()+imu()[:-1]+bytes([imu()[-1]^1])):
            with self.subTest(raw=raw):
                with self.assertRaises(ValueError):
                    parser=FrameDecoder(codec());parser.feed(raw,1);parser.finish()

    def test_foreign_source_and_arming_are_rejected_from_actual_wire(self):
        for raw in (heartbeat(2),heartbeat(armed=True)):
            with self.assertRaises(ValueError):FrameDecoder(codec()).feed(raw,1)

    def test_changed_or_missing_retained_decoded_rows_are_rejected(self):
        raw=heartbeat()+imu();parser=FrameDecoder(codec());rows,_=parser.feed(raw,1)
        for candidate in (list(reversed(rows)),rows[:-1],copy.deepcopy(rows)):
            if candidate==rows:candidate[1]['xmag']=9
            with self.assertRaises(ValueError):replay(raw,candidate,codec)

    def test_actual_receive_clock_must_remain_finite_and_ordered(self):
        for at in (float('nan'),float('inf'),-1):
            with self.assertRaises(ValueError):FrameDecoder(codec()).feed(heartbeat(),at)
        parser=FrameDecoder(codec());parser.feed(heartbeat(),2)
        with self.assertRaises(ValueError):parser.feed(imu(),1)

    def test_all_scaled_wire_families_keep_fields(self):
        output=codec();raw=heartbeat()
        for family in ('scaled_imu','scaled_imu2','scaled_imu3'):
            raw+=bytes(getattr(output,family+'_encode')(1234,1000,-2000,0,1000,-2000,125,1000,-250,0,temperature=-4000).pack(output))
        parser=FrameDecoder(codec());rows,_=parser.feed(raw,2)
        self.assertEqual([r['mavpackettype'] for r in rows[1:]],['SCALED_IMU','SCALED_IMU2','SCALED_IMU3'])
        self.assertTrue(all(r['xmag']==1000 and r['temperature']==-4000 for r in rows[1:]));self.assertEqual(parser.finish()['frames'],4)

    def test_decoded_memory_bound_cannot_be_relaxed(self):
        for invalid in (True,0,20001):
            with self.assertRaises(ValueError):FrameDecoder(codec(),max_frames=invalid)
        parser=FrameDecoder(codec(),max_frames=1)
        with self.assertRaisesRegex(ValueError,'bound exceeded'):parser.feed(heartbeat()+imu(),1)


if __name__=='__main__':unittest.main(verbosity=2)
