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

# started：该步发起调用的时刻（run_sequence 给了 clock 才有，否则为 None）。
StepResult = collections.namedtuple('StepResult', ['name', 'response', 'error', 'started'],
                                    defaults=(None,))


def parse_mode(mode):
    try:
        return MODES[mode]
    except (KeyError, TypeError):
        raise ValueError('mode must be one of %s, got %r' % ('/'.join(sorted(MODES)), mode))


def run_sequence(steps, calls, clock=None):
    """按 steps 顺序调用 calls[name]()；单步抛异常只记录，不影响后续步骤。

    给了 clock 时每步发起前取一次时间记入 started（both 模式下读码器要等相机读完才开始）。
    """
    results = []
    for name in steps:
        started = clock() if clock is not None else None
        try:
            results.append(StepResult(name, calls[name](), '', started))
        except Exception as error:  # 服务不在线 / 调用失败
            results.append(StepResult(name, None, str(error) or type(error).__name__, started))
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


class Raw(str):
    """原样输出、不加引号的值（枚举词、已按位数格式化的数字）；普通 str 值输出时加双引号。"""


def step_fields(result):
    """一路读码结果 -> 有序 [(key, value)]，供 format_block 排版。"""
    if result.response is None:
        return [('ok', False), ('error', Raw('unavailable')), ('detail', result.error)]
    res = result.response
    fields = [('ok', bool(res.success))]
    if result.name == 'camera':
        if not res.success:
            fields.append(('error', Raw(res.error or ('partial' if res.partial else 'fail'))))
        if res.report:
            fields.append(('report', res.report))
        if res.codes:
            fields.append(('codes', ['%s:%s' % (c.format, c.code) for c in res.codes]))
    elif res.success:
        fields.append(('code', res.code))
    else:
        fields.append(('error', Raw(res.error)))
    fields.append(('ms', int(round(res.elapsed_s * 1000.0))))
    return fields


def summary_is_warning(results):
    """读码结果本身（读到/读不到）都是正常结果打 INFO；只有服务不在线才算异常打 WARN。"""
    return any(result.response is None for result in results)


def _value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, Raw):
        return str(value)
    if isinstance(value, str):
        return '"%s"' % escape(value)
    return str(value)


def format_block(title, sections):
    """rostopic echo 风格的多行块：标题行 + 每段 `name:` 下缩进两格的 key: value，结尾 `---`。

    sections 为 [(name, fields)]；fields 为 None 时输出 `name: none`。
    """
    lines = [title]
    for name, fields in sections:
        if fields is None:
            lines.append('%s: none' % name)
            continue
        lines.append('%s:' % name)
        for key, value in fields:
            if isinstance(value, (list, tuple)):
                if not value:
                    lines.append('  %s: []' % key)
                    continue
                lines.append('  %s:' % key)
                lines.extend('    - %s' % _value(item) for item in value)
            else:
                lines.append('  %s: %s' % (key, _value(value)))
    lines.append('---')
    return '\n'.join(lines)
