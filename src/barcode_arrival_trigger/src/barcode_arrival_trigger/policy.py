"""POS_HOLD 跳变 -> 单次读码的纯状态机（不依赖 ROS、不做 IO），以及到位门控判据。

行为：
  * 只认「别的模式 -> POS_HOLD」的跳变；节点启动后收到的第一条只作基线，
    所以启动瞬间已经停在 POS_HOLD 不会补读；
  * 停在 POS_HOLD 期间不重读，退出后再进入才再读一次；一次读码失败也不重试；
  * 上一次读码还没结束时出现的新跳变按忙时忽略（照样占用到位序号，但不读）；
  * 读码期间退出 POS_HOLD 不取消本次读码，只作标记，由 finish() 取走。
到位只由 front_marker_control 的 control_mode 决定，本模块不做视觉或定位判断。
"""
import collections
import math

POS_HOLD = 'POS_HOLD'
# 到位读码要求飞控处于该模式。
REQUIRED_MODE = 'OFFBOARD'

ArrivalDecision = collections.namedtuple('ArrivalDecision', ['fire_seq', 'ignored_seq'])


class ArrivalHoldTrigger(object):
    def __init__(self):
        self._mode = None
        self._seq = 0
        self._busy = False
        self._hold_lost = False

    @property
    def mode(self):
        """最近一次收到的 control_mode；None 表示一条都还没收到。"""
        return self._mode

    @property
    def busy(self):
        """True 表示上一次到位读码还没结束。"""
        return self._busy

    @property
    def seq(self):
        """已经出现过的到位序号（含被忙时忽略的跳变）。"""
        return self._seq

    def update(self, mode):
        """喂一次 control_mode，返回该次跳变对应的到位序号。

        fire_seq：需要读码的到位序号；ignored_seq：跳变成立但上一次读码尚未结束。
        """
        previous = self._mode
        self._mode = mode
        if previous is None:
            # 第一条（通常是 latch 到的那一刻的模式）只作基线：不补读、不占号。
            return ArrivalDecision(None, None)
        if previous == POS_HOLD:
            if mode != POS_HOLD and self._busy:
                # 读码还没结束就离开 POS_HOLD：不取消读码，只打标。
                self._hold_lost = True
            return ArrivalDecision(None, None)
        if mode != POS_HOLD:
            return ArrivalDecision(None, None)
        self._seq += 1
        if self._busy:
            return ArrivalDecision(None, self._seq)
        self._busy = True
        self._hold_lost = False
        return ArrivalDecision(self._seq, None)

    def finish(self):
        """读码结束：释放忙标记，并返回本次读码期间是否退出过 POS_HOLD。"""
        hold_lost = self._hold_lost
        self._busy = False
        self._hold_lost = False
        return hold_lost


def gate_reason(connected, armed, mode, state_age_s, state_timeout_s):
    """到位跳变那一刻的飞行门控：返回 '' 表示通过，否则返回放弃本次到位的理由。"""
    if state_age_s is None:
        return '尚未收到 /mavros/state'
    age = float(state_age_s)
    if not math.isfinite(age) or age < 0.0:
        return 'state 接收时间异常'
    if age > float(state_timeout_s):
        return 'state 超时 %.1fs' % age
    if not connected:
        return '未连接'
    if not armed:
        return '未解锁'
    if mode != REQUIRED_MODE:
        return '非 %s (%s)' % (REQUIRED_MODE, mode)
    return ''


def marker_id_within(marker_id, age_s, timeout_s):
    """跳变时刻可用的 marker id：debug 消息须在 timeout_s 以内，否则返回 -1。

    到位读码关心的是「到位那一刻看见的是哪个 marker」，所以过期消息不参与，
    避免用上一次悬停或上一段航线残留的 id 冒充本次到位。
    """
    if age_s is None:
        return -1
    age = float(age_s)
    if not math.isfinite(age) or age < 0.0 or age > float(timeout_s):
        return -1
    return int(marker_id)
