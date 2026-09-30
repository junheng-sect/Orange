"""一次拨杆对应的读码流程和结果汇总（不依赖 ROS，服务调用由节点注入）。

both 模式固定先相机后读码器，两个都执行（不是失败才兜底），同一次拨杆拿两份结果对比。
先相机：相机节点收到请求后等下一帧新图，读码器的补光/瞄准光不会拍进这帧；
与 DroneVision camera_then_serial 顺序一致。
"""
import collections

MODES = {
    'camera': ('camera',),
    'serial': ('serial',),
    'both': ('camera', 'serial'),
}

StepResult = collections.namedtuple('StepResult', ['name', 'response', 'error'])


def parse_mode(mode):
    try:
        return MODES[mode]
    except (KeyError, TypeError):
        raise ValueError('mode must be one of %s, got %r' % ('/'.join(sorted(MODES)), mode))


def run_sequence(steps, calls):
    """按 steps 顺序调用 calls[name]()；单步抛异常只记录，不影响后续步骤。"""
    results = []
    for name in steps:
        try:
            results.append(StepResult(name, calls[name](), ''))
        except Exception as error:  # 服务不在线 / 调用失败
            results.append(StepResult(name, None, str(error) or type(error).__name__))
    return results


def step_ok(result):
    return result.response is not None and bool(result.response.success)


def escape(text):
    out = []
    for ch in text:
        if ch in '"\\':
            out.append('\\' + ch)
        elif ord(ch) < 0x20:
            out.append('\\x%02x' % ord(ch))
        else:
            out.append(ch)
    return ''.join(out)


def format_step(result):
    if result.response is None:
        return '%s: unavailable (%s)' % (result.name, result.error)
    if result.name == 'camera':
        return _format_camera(result.response)
    return _format_serial(result.response)


def _format_camera(res):
    state = 'ok' if res.success else ('partial' if res.partial else 'fail')
    parts = ['camera: %s' % state]
    if res.error and res.error != state:
        parts.append('error=%s' % res.error)
    if res.report:
        parts.append('report="%s"' % res.report)
    parts.extend('[%s]"%s"' % (c.format, escape(c.code)) for c in res.codes)
    parts.append('%.0fms' % (res.elapsed_s * 1000.0))
    return ' '.join(parts)


def _format_serial(res):
    if res.success:
        return 'serial: ok "%s" %.0fms' % (escape(res.code), res.elapsed_s * 1000.0)
    return 'serial: fail error=%s %.0fms' % (res.error, res.elapsed_s * 1000.0)
