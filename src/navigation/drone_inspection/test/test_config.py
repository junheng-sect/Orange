import copy
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import yaml
from drone_inspection.config import load, segments


class ConfigTest(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'task.yaml'
        self.base = yaml.safe_load((Path(__file__).resolve().parents[1] / 'missions/indoor_shelf.yaml').read_text())

    def parse(self, cfg):
        self.path.write_text(yaml.safe_dump(cfg))
        return load(self.path)

    def test_limits_and_mixed_segments(self):
        cfg = self.parse(self.base)
        self.assertEqual((cfg['flight']['max_vel'],cfg['flight']['planner_max_vel']), (.5,.4))
        self.assertEqual(cfg['recording']['save_frames'], 'off')
        pts = [dict(action=k) for k in ('pass','pass','hover','scan_shelf','pass')]
        self.assertEqual(list(segments(pts)), [[0,1,2],[3],[4]])

    def test_invalid_missions_fail_before_ros(self):
        for section, field, value in [('map','sha256',''), ('flight','auto_takeoff',False),
                ('flight','return_home',False), ('flight','max_vel',.3), ('flight','max_acc_z',True),
                ('flight','bounds_max',[-4,3,3]), ('flight','max_control_z',1.2),
                ('recording','save_frames',False)]:
            with self.subTest(section=section, field=field):
                cfg = copy.deepcopy(self.base)
                cfg[section][field] = value
                with self.assertRaises(ValueError):
                    self.parse(cfg)
        for change in [dict(xyz=[0,float('nan'),1]), dict(action='pass'), dict(action='unknown'), dict(hold_s=2)]:
            cfg = copy.deepcopy(self.base)
            cfg['waypoints'][0].update(change)
            with self.assertRaises(ValueError):
                self.parse(cfg)


if __name__ == '__main__':
    unittest.main()
