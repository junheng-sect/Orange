"""Compare nominal indoor6 and mission records from independent loopback master runs."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def records(folder):
    flights, goals = list(folder.glob('flight_*.json')), list(folder.glob('goal_*.json'))
    if len(flights) != 1 or len(goals) != 1:
        raise ValueError('需要一个仅含单次运行结果的目录：' + str(folder))
    return json.loads(flights[0].read_text()), json.loads(goals[0].read_text())


def compare(old, new):
    flights, goals = zip(records(old), records(new))
    assert all(f['end'] == 'SUCCEEDED' for f in (*flights, *goals))
    orders = [[batch['order'] for batch in goal['batches']] for goal in goals]
    assert orders[0] == orders[1] == [[5],[2,1],[4,0],[3]]
    legs = [g['legs'] for g in goals]
    assert len(legs[0]) == len(legs[1]) == 12
    errors = []
    for a,b in zip(*legs):
        assert a['direction'] == b['direction'] and a['end'] == b['end'] == 'SUCCEEDED'
        np.testing.assert_allclose(a['goal_yaw_map_deg'], b['goal_yaw_map_deg'], atol=1.e-6)
        errors.append(float(np.linalg.norm(np.subtract(a['goal_map'], b['goal_map']))))
    assert max(errors) < .005
    np.testing.assert_allclose(goals[0]['return_map'], goals[1]['return_map'], atol=1.e-6)
    scanned = [[(s['id'],s['status'],s.get('qr')) for s in g['scans']] for g in goals]
    assert scanned[0] == scanned[1] and len(scanned[0]) == 6
    for flight in flights:
        counts = Counter(s['service'] for s in flight['services'])
        assert counts['takeoff'] == counts['land'] == 1
    return dict(order=orders[0], legs=len(legs[0]), max_goal_difference_m=max(errors),
                return_map=goals[0]['return_map'], scans=scanned[0], takeoff_requests=1, land_requests=1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('old',type=Path)
    parser.add_argument('new',type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.old,args.new), ensure_ascii=False, indent=2))
