"""Parameters shared by the indoor vertical test, its CLI and the web dispatcher."""
import argparse

TASK = 'test_indoor_takeoff_land'
MAP = '20261114_101654_indoor_fly_raw_000001'
MAX_MAP_Z = 1.70
PARAMETERS = {
    'height': dict(default=.50, min=.30, max=2.50, label='起飞高度', unit='m'),
    'hover': dict(default=3., min=0., max=30., label='额外悬停', unit='s'),
    'delay': dict(default=3., min=1., max=4., label='解锁倒计时', unit='s'),
}


def validate(values):
    if not isinstance(values, dict) or values.keys() != PARAMETERS.keys():
        raise ValueError('原地起降需要完整的高度、悬停时间和倒计时')
    for key, spec in PARAMETERS.items():
        value = values[key]
        if type(value) not in (float, int) or not spec['min'] <= value <= spec['max']:
            raise ValueError('%s 应在 %g–%g %s 范围内' %
                             (spec['label'], spec['min'], spec['max'], spec['unit']))
    return values


def arguments(parser):
    for key, spec in PARAMETERS.items():
        parser.add_argument('--' + key, type=float, default=spec['default'], help=spec['label'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    arguments(parser)
    try:
        values = validate(vars(parser.parse_args()))
    except ValueError as error:
        parser.error(str(error))
    print(*(values[k] for k in PARAMETERS))
