"""Process-ownership regressions with mocked process tables and signals only."""
import subprocess
import unittest
from unittest.mock import Mock, patch

import owned_processes as owned


def row(pid, parent, command):
    return {'pid': pid, 'parent': parent, 'group': pid, 'command': command}


def process(live, args=None):
    child = Mock(pid=100, args=args or ['/test/probe', 'private-port'])
    child.poll.return_value = None if live else 0
    return child


class OwnershipTests(unittest.TestCase):
    def test_reaped_pid_reuse_does_not_claim_replacement_or_descendants(self):
        table = {100: row(100, 1, '/unrelated/app'), 101: row(101, 100, '/unrelated/child')}
        with patch.object(owned, 'snapshot', return_value=table):
            self.assertEqual(owned.owned_records(process(False)), [])

    def test_live_child_claims_only_its_descendants(self):
        table = {100: row(100, 1, '/test/probe private-port'),
                 101: row(101, 100, '/test/child'), 102: row(102, 101, '/test/grandchild'),
                 103: row(103, 1, '/unrelated/app')}
        with patch.object(owned, 'snapshot', return_value=table):
            self.assertEqual({r['pid'] for r in owned.owned_records(process(True))}, {100, 101, 102})

    def test_reaped_antler_reconciles_only_exact_private_mission(self):
        table = {100: row(100, 1, '/unrelated/app'),
                 101: row(101, 1, '/bin/MOOSDB /private/case/mission.moos'),
                 102: row(102, 1, '/bin/MOOSDB /private/other/mission.moos'),
                 103: row(103, 1, '/bin/other /private/case/mission.moos')}
        with patch.object(owned, 'snapshot', return_value=table):
            records = owned.owned_records(process(False, ['/bin/pAntler', '/private/case/mission.moos']))
            self.assertEqual({r['pid'] for r in records}, {101})

    def test_cleanup_never_signals_reused_reaped_pid(self):
        table = {100: row(100, 1, '/unrelated/app')}
        with patch.object(owned, 'snapshot', return_value=table), \
             patch.object(owned.os, 'kill') as kill, patch.object(owned.os, 'killpg') as killpg:
            self.assertEqual(owned.stop_process_tree(process(False)), [])
            kill.assert_not_called()
            killpg.assert_not_called()

    def test_changed_child_identity_is_not_signalled_after_parent_exit(self):
        initial = {100: row(100, 1, '/test/probe private-port'), 101: row(101, 100, '/test/child')}
        replacement = {101: row(101, 1, '/unrelated/replacement')}
        child = process(True)
        with patch.object(owned, 'snapshot', side_effect=[initial, replacement, replacement]), \
             patch.object(owned.os, 'kill') as kill, patch.object(owned.os, 'killpg') as killpg:
            self.assertEqual(owned.stop_process_tree(child), [])
            kill.assert_not_called()
            self.assertEqual(killpg.call_count, 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
