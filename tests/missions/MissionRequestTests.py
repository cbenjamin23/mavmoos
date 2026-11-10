#!/usr/bin/env python3
"""Standard-library tests for bounded mission request generation; no I/O service."""
from pathlib import Path
import tempfile
import unittest

from mission_request import request


class RequestTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'plan.waypoints'
        self.rows = ['0 1 0 16 0 0 0 0 42.3584560 -71.0875890 10 1',
                     '1 0 3 16 0 0 0 0 42.3584660 -71.0875890 5 1']

    def upload(self, rows=None, header='QGC WPL 110'):
        self.path.write_text(header + '\n' + '\n'.join(self.rows if rows is None else rows) + '\n')
        return request('upload-1', 'upload', self.path, 20)

    def test_read_clear_cancel_do_not_require_files(self):
        for action in ('download', 'clear', 'cancel'):
            self.assertEqual(request('request-1', action), f'id=request-1,action={action}')
        self.assertEqual(request('timed', 'download', timeout=2), 'id=timed,action=download,timeout=2')

    def test_coordinates_scaled_and_current_metadata_retained(self):
        result = self.upload()
        self.assertEqual(result, 'id=upload-1,action=upload,timeout=20,items=0:16:0:1:1:0:0:0:0:423584560:-710875890:10|1:16:3:0:1:0:0:0:0:423584660:-710875890:5')

    def test_deprecated_global_frames_canonicalized_without_changing_scaling(self):
        self.rows[0] = self.rows[0].replace('0 1 0 16', '0 1 5 16')
        self.rows[1] = self.rows[1].replace('1 0 3 16', '1 0 6 16')
        self.assertIn('items=0:16:0:1:1:', self.upload())
        self.assertIn('|1:16:3:0:1:', self.upload())

    def test_coordinate_rounding_explicit_half_away_from_zero(self):
        self.rows[1] = self.rows[1].replace('42.3584660 -71.0875890', '42.35846605 -71.08758905')
        self.assertIn(':423584661:-710875891:5', self.upload())

    def test_unsupported_actions_ids_and_timeouts_reject(self):
        for operation_id in ('', 'has,comma', '../path', 'x' * 49, 'é'):
            with self.subTest(operation_id=operation_id), self.assertRaises(ValueError):
                request(operation_id, 'download')
        for value in (0, 121, float('nan'), float('inf')):
            with self.subTest(timeout=value), self.assertRaises(ValueError):
                request('x', 'download', timeout=value)
        with self.assertRaises(ValueError):
            request('x', 'start')
        with self.assertRaises(ValueError):
            request('x', 'cancel', timeout=1)

    def test_file_only_allowed_for_upload_and_is_required(self):
        with self.assertRaises(ValueError):
            request('x', 'download', self.path)
        with self.assertRaises(ValueError):
            request('x', 'upload')

    def test_home_and_contiguous_sequence_required(self):
        for rows in ([], self.rows[1:], [self.rows[0], self.rows[1].replace('1 0 3', '2 0 3')]):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.upload(rows)
        with self.assertRaises(ValueError):
            self.upload([self.rows[0].replace('0 1 0', '0 1 3'), self.rows[1]])

    def test_shape_header_guided_and_unsupported_commands_reject(self):
        with self.assertRaises(ValueError):
            self.upload(header='not a mission')
        for row in (self.rows[1] + ' extra', self.rows[1].replace('1 0 3 16', '1 2 3 16'),
                    self.rows[1].replace('1 0 3 16', '1 0 3 183'), self.rows[1].replace('1 0 3 16', '1 0 1 16')):
            with self.subTest(row=row), self.assertRaises(ValueError):
                self.upload([self.rows[0], row])

    def test_nonfinite_and_out_of_profile_values_reject(self):
        for before, after in [('42.3584660', 'nan'), ('42.3584660', '91'), ('-71.0875890', '181'),
                              (' 5 1', ' 20001 1'), ('16 0 0 0 0', '16 0.5 0 0 0'), ('16 0 0 0 0', '16 0 0 0 1')]:
            with self.subTest(after=after), self.assertRaises(ValueError):
                self.upload([self.rows[0], self.rows[1].replace(before, after)])

    def test_supported_land_takeoff_and_rtl_shapes(self):
        for command in (21, 22):
            self.assertIn(f'|1:{command}:3:', self.upload([self.rows[0], self.rows[1].replace('1 0 3 16', f'1 0 3 {command}')]))
        self.assertIn('|1:20:2:0:1:0:0:0:0:0:0:0', self.upload([self.rows[0], '1 0 2 20 0 0 0 0 0 0 0 1']))

    def test_table_limit_stops_before_unbounded_request(self):
        rows = [self.rows[0]] + [self.rows[1].replace('1 0 3', f'{i} 0 3') for i in range(1, 4097)]
        with self.assertRaises(ValueError):
            self.upload(rows)


if __name__ == '__main__':
    unittest.main(verbosity=2)
