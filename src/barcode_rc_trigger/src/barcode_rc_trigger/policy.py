"""RC 弹簧开关 -> 单次触发的纯状态机（不依赖 ROS、不做 IO）。

行为复刻 9.1 SEEKER 上的 CH9 拍照节点 rc_photo_capture：
  * PWM 低于 arm_below 才布防；
  * 布防后 PWM 高于 trigger_above 连续 hold_s 触发一次，随即撤防；
  * 回到低位后再次布防，所以一次拨杆只触发一次，按住不放也不会重复触发；
  * 中间区（arm_below..trigger_above）不布防也不触发，只清高位计时；
  * 上电或 RC 重连时开关已在高位不会触发（必须先见到低位）。
RC 无效（通道缺失、数值越界、话题超时）时撤防并清计时。
"""


class RcPulseTrigger(object):
    def __init__(self, arm_below=1600, trigger_above=1700, hold_s=0.05):
        if not arm_below < trigger_above:
            raise ValueError('arm_below must be lower than trigger_above')
        if not hold_s >= 0.0:
            raise ValueError('hold_s must be >= 0')
        self.arm_below = arm_below
        self.trigger_above = trigger_above
        self.hold_s = float(hold_s)
        self._armed = False
        self._high_since = None

    def update(self, now, pwm):
        """喂一次采样；pwm=None 表示 RC 无效。返回 True 表示这次采样触发。"""
        if pwm is None:
            self.reset()
            return False
        if pwm < self.arm_below:
            self._armed = True
            self._high_since = None
            return False
        if pwm <= self.trigger_above or not self._armed:
            self._high_since = None
            return False
        if self._high_since is None:
            self._high_since = now
        if now - self._high_since < self.hold_s:
            return False
        self._armed = False
        self._high_since = None
        return True

    def reset(self):
        self._armed = False
        self._high_since = None

    @property
    def armed(self):
        return self._armed
