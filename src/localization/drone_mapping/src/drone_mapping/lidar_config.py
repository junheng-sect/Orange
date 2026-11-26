"""Read the board and sensor addresses from the same JSON as the MID-360S driver."""
from ipaddress import IPv4Address
import json
import os
from pathlib import Path
import sys


def network_addresses(workspace):
    default = Path(workspace) / 'src/drivers/livox_ros_driver2/config/MID360s_config.local.json'
    path = Path(os.environ.get('MID360S_LIVOX_CONFIG', default))
    with path.open() as stream:
        config = json.load(stream)
    try:
        host = config['Mid360s']['host_net_info'][0]['host_ip']
        sensor = config['lidar_configs'][0]['ip']
        return str(IPv4Address(host)), str(IPv4Address(sensor))
    except (KeyError, IndexError, TypeError, ValueError) as error:
        raise ValueError('Invalid MID-360S addresses in %s' % path) from error


if __name__ == '__main__':
    try:
        print(*network_addresses(sys.argv[1]))
    except (OSError, ValueError) as error:
        sys.exit(str(error))
