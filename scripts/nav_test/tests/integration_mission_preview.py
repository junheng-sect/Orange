"""Run the warehouse preset through the actual EGO, executor and output-disabled gateway."""
import json
from pathlib import Path
import numpy as np
import rospkg

from integration_indoor4 import main
from integration_indoor5 import CornerCamera
from nav_indoor5 import load_indoor5_route
from drone_inspection.config import load
from drone_inspection.records import Records
from drone_inspection.route import MissionRoute
from drone_inspection.runtime import arguments


class Camera(CornerCamera):
    target_topic = '/drone_inspection/target_tag_pose'


def runner(args, cfg, *clients):
    out = Path(args.out)
    mission = load(Path(rospkg.RosPack().get_path('drone_inspection')) / 'missions/indoor_shelf.yaml')
    mission['navigation'] = cfg
    mission['recording']['capture_waypoints'] = args.capture_waypoints
    return MissionRoute(arguments(mission, out, out, Records(out, out.name, mission, preview=True), True), cfg, *clients)


def check(result, camera, sent, route, fixture, out):
    assert [b['order'] for b in result.record['batches']] == [[5], [2,1], [4,0], [3]]
    assert len(sent) == 12 and set(camera.ids) == set(range(6))
    np.testing.assert_allclose(result.record['legs'][-1]['goal_map'], [.45,.34,.48], atol=.005)
    assert json.loads((out/'mission_state.json').read_text())['end'] == 'SUCCEEDED'


if __name__ == '__main__':
    main(Camera, runner, load_indoor5_route, check, 11452)
