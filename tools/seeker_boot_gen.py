#!/usr/bin/env python3
"""seeker_boot.yaml -> SEEKER_ENABLE_* 环境变量（输出 KEY=value 行）。

供 tools/seeker-boot-start 在 seeker.service 启动时调用：
stdout 逐行输出 KEY=value，由 wrapper export 后传给 roslaunch 的 $(optenv)。
校验失败（未知键/非布尔值/YAML 语法错误）时非零退出并输出原因，
wrapper 随之退出，服务启动失败——避免带错配置半启动。
"""

from pathlib import Path
import sys

import yaml

# YAML 键 -> 环境变量名。默认值与 seeker_autostart.launch 中 optenv 缺省一致。
FLAGS = {
    'base_camera_pipeline': ('SEEKER_ENABLE_BASE', True),
    'stereo_depth': ('SEEKER_ENABLE_DEPTH', True),
    'vins': ('SEEKER_ENABLE_VINS', True),
    'mavros': ('SEEKER_ENABLE_MAVROS', True),
    'front_marker_tracking': ('SEEKER_ENABLE_FRONT_MARKER', True),
    'rc_bag_recorder': ('SEEKER_ENABLE_RC_BAG', True),
    'rc_photo_capture': ('SEEKER_ENABLE_RC_PHOTO', True),
    'vz_chirp_control': ('SEEKER_ENABLE_VZ_CHIRP', False),
    'vz_step_control': ('SEEKER_ENABLE_VZ_STEP', False),
}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().with_name('seeker_boot.yaml')
    try:
        with open(path) as handle:
            data = yaml.safe_load(handle)
    except FileNotFoundError:
        return 0  # 文件缺失 = 全部按 launch 内置默认值（全量启动）
    except yaml.YAMLError as exception:
        print('seeker_boot.yaml 语法错误: %s' % exception, file=sys.stderr)
        return 1
    if data is None:
        return 0
    if not isinstance(data, dict):
        print('seeker_boot.yaml 顶层应为键值表', file=sys.stderr)
        return 1

    unknown = sorted(set(data) - set(FLAGS))
    if unknown:
        print('seeker_boot.yaml 含未知键（可能拼写错误）: %s；合法键: %s'
              % (', '.join(map(str, unknown)), ', '.join(FLAGS)), file=sys.stderr)
        return 1

    for key, value in data.items():
        if not isinstance(value, bool):
            print('seeker_boot.yaml 的 %s 必须是 true/false（收到 %r）'
                  % (key, value), file=sys.stderr)
            return 1
    for key, (env_name, _default) in FLAGS.items():
        if key in data:
            print('%s=%s' % (env_name, 'true' if data[key] else 'false'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
