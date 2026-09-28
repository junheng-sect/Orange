#!/usr/bin/env python3
import unittest

from tether_winch_control.controller import Config
from tether_winch_control.simulation import run_scenario


class ScenarioTests(unittest.TestCase):
    def test_end_to_end_carry_scenario(self):
        rows = run_scenario()
        grouped = {}
        for row in rows:
            grouped.setdefault(row['phase'], []).append(row)
        for phase in ('depart', 'stop_after_depart', 'constant_radius', 'explicit_reenable'):
            self.assertTrue(all(r['mode'] == 'LOW' for r in grouped[phase]), phase)
        for phase in ('approach', 'reverse_in', 'approach_before_dropout'):
            self.assertEqual(grouped[phase][-1]['mode'], 'HIGH', phase)
        for phase in ('stop_after_approach', 'reverse_out'):
            self.assertEqual(grouped[phase][-1]['mode'], 'LOW', phase)
        self.assertEqual(grouped['dropout'][-1]['mode'], 'FAULT')
        self.assertTrue(all(r['mode'] == 'FAULT' for r in grouped['recover_without_enable']))
        self.assertTrue(grouped['recover_without_enable'][-1]['input_valid'])
        for row in rows:
            if row['mode'] in ('DISABLED', 'FAULT'):
                self.assertIsNone(row['target_duty'])
        self.assertEqual({r['target_duty'] for r in rows if r['enabled']}, {0.2, 0.4})

    def test_configurable_duties_do_not_change_classification(self):
        original = run_scenario()
        changed = run_scenario(Config(low_duty=0.1, high_duty=0.3))
        self.assertEqual([r['mode'] for r in original], [r['mode'] for r in changed])
        self.assertEqual({r['target_duty'] for r in changed if r['enabled']}, {0.1, 0.3})


if __name__ == '__main__':
    unittest.main()
