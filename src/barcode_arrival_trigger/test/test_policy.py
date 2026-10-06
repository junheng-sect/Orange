#!/usr/bin/env python3
import unittest

from barcode_arrival_trigger.policy import (ArrivalHoldTrigger, POS_HOLD,
                                            gate_reason, marker_id_within)

ALIGN, RETREAT, SAMPLING = 'ALIGN', 'RETREAT', 'YAW_SAMPLING'


class ArrivalHoldTriggerTest(unittest.TestCase):
    def setUp(self):
        self.trigger = ArrivalHoldTrigger()

    def fire(self, *modes):
        """按顺序喂模式，返回其中触发的到位序号。"""
        seqs = []
        for mode in modes:
            decision = self.trigger.update(mode)
            if decision.fire_seq is not None:
                seqs.append(decision.fire_seq)
        return seqs

    def test_first_mode_is_baseline_only(self):
        # 启动时 latch 到的 POS_HOLD 不补读、不占号。
        self.assertEqual(self.fire(POS_HOLD), [])
        self.assertEqual(self.trigger.seq, 0)
        # 之后从别的模式再进入 POS_HOLD 才读。
        self.assertEqual(self.fire(ALIGN, POS_HOLD), [1])
        self.assertEqual(self.trigger.seq, 1)

    def test_first_mode_align_then_hold_fires(self):
        self.assertEqual(self.fire(ALIGN), [])
        self.assertEqual(self.fire(POS_HOLD), [1])

    def test_hold_edge_only(self):
        self.fire(ALIGN)
        # 停在 POS_HOLD 期间重复发布不触发。
        self.assertEqual(self.fire(POS_HOLD, POS_HOLD), [1])
        self.trigger.finish()
        # 别的模式之间来回不触发；再进入 POS_HOLD 才触发。
        self.assertEqual(self.fire(RETREAT, ALIGN, SAMPLING), [])
        self.assertEqual(self.fire(POS_HOLD), [2])

    def test_every_reentry_fires_once(self):
        self.trigger.finish()
        self.fire(ALIGN)
        seqs = []
        for _ in range(3):
            seqs += self.fire(POS_HOLD)
            self.trigger.finish()   # 读码结束，允许下一次到位
            seqs += self.fire(ALIGN)
        self.assertEqual(seqs, [1, 2, 3])

    def test_busy_edge_ignored_but_takes_seq(self):
        self.fire(ALIGN, POS_HOLD)
        self.assertTrue(self.trigger.busy)
        # 上一次读码还没结束：跳变成立但被忽略，仍然占用序号。
        decision = self.trigger.update(ALIGN)
        self.assertEqual((decision.fire_seq, decision.ignored_seq), (None, None))
        decision = self.trigger.update(POS_HOLD)
        self.assertEqual((decision.fire_seq, decision.ignored_seq), (None, 2))
        self.assertEqual(self.trigger.seq, 2)
        self.assertTrue(self.trigger.busy)
        # 读码结束后再进入才读，序号继续往后走。
        self.trigger.finish()
        self.fire(ALIGN)
        self.assertEqual(self.fire(POS_HOLD), [3])

    def test_finish_releases_busy(self):
        self.fire(ALIGN, POS_HOLD)
        self.assertFalse(self.trigger.finish())
        self.assertFalse(self.trigger.busy)
        self.assertFalse(self.trigger.finish())

    def test_hold_lost_only_during_read(self):
        # 停在 POS_HOLD 期间的模式变化与读码无关：不置位。
        self.fire(ALIGN, POS_HOLD)
        self.trigger.finish()
        self.assertEqual(self.trigger.update(RETREAT).fire_seq, None)
        self.assertFalse(self.trigger.finish())
        # 读码期间退出 POS_HOLD：标记，读完取走并清零。
        self.fire(ALIGN, POS_HOLD)
        self.trigger.update(RETREAT)
        self.assertTrue(self.trigger.finish())
        self.assertFalse(self.trigger.finish())
        # 下一次读码重新开始计数。
        self.fire(ALIGN, POS_HOLD)
        self.assertFalse(self.trigger.finish())

    def test_hold_lost_cleared_by_new_read(self):
        self.fire(ALIGN, POS_HOLD)
        self.trigger.update(ALIGN)
        self.trigger.finish()
        # 新的一次读码开始时清掉上一次的标记。
        self.fire(POS_HOLD)
        self.assertFalse(self.trigger.finish())

    def test_mode_property(self):
        self.assertIsNone(self.trigger.mode)
        self.fire(ALIGN)
        self.assertEqual(self.trigger.mode, ALIGN)
        self.fire(POS_HOLD)
        self.assertEqual(self.trigger.mode, POS_HOLD)


class GateReasonTest(unittest.TestCase):
    def reason(self, connected=True, armed=True, mode='OFFBOARD', age=0.1, timeout=2.0):
        return gate_reason(connected, armed, mode, age, timeout)

    def test_pass(self):
        self.assertEqual(self.reason(), '')
        # 刚好等于超时门限不算超时。
        self.assertEqual(self.reason(age=2.0), '')

    def test_state_missing_or_stale(self):
        self.assertIn('尚未收到', self.reason(age=None))
        self.assertIn('超时', self.reason(age=2.5))
        self.assertIn('异常', self.reason(age=-0.5))
        self.assertIn('异常', self.reason(age=float('nan')))

    def test_flight_state_required(self):
        self.assertIn('未连接', self.reason(connected=False))
        self.assertIn('未解锁', self.reason(armed=False))
        self.assertIn('OFFBOARD', self.reason(mode='AUTO.RTL'))
        self.assertIn('AUTO.RTL', self.reason(mode='AUTO.RTL'))


class MarkerIdTest(unittest.TestCase):
    def test_fresh_message_is_used(self):
        self.assertEqual(marker_id_within(7, 0.1, 0.5), 7)
        # 刚好等于门限仍算新鲜。
        self.assertEqual(marker_id_within(0, 0.5, 0.5), 0)

    def test_missing_or_stale_becomes_minus_one(self):
        self.assertEqual(marker_id_within(7, None, 0.5), -1)
        self.assertEqual(marker_id_within(7, 0.51, 0.5), -1)
        self.assertEqual(marker_id_within(7, 30.0, 0.5), -1)
        self.assertEqual(marker_id_within(7, -0.1, 0.5), -1)
        self.assertEqual(marker_id_within(7, float('nan'), 0.5), -1)

    def test_id_is_passed_through(self):
        self.assertEqual(marker_id_within(-1, 0.0, 0.5), -1)
        self.assertEqual(marker_id_within(12, 0.2, 0.5), 12)


if __name__ == '__main__':
    unittest.main()
