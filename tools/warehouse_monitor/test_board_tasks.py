from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import board_tasks


class IndoorDispatchTests(unittest.TestCase):
    def test_map_frame_comes_from_selected_script_workspace(self):
        with tempfile.TemporaryDirectory() as tmp:
            workspace = Path(tmp) / 'Orange checkout'
            scripts = workspace / 'scripts'
            frame = workspace / 'src/localization/car_bridge/config/map_frame_indoor_fly.yaml'
            frame.parent.mkdir(parents=True)
            frame.touch()
            with patch.object(board_tasks, 'SCRIPTS', scripts):
                for task_id in ('test_indoor4', 'test_indoor5', 'test_indoor6'):
                    args = board_tasks.script_command({'id': task_id, 'mapId': 'indoor'})
                    self.assertEqual(args[0], str(scripts / f'nav_{task_id}.sh'))
                    self.assertEqual(args[args.index('--map-frame') + 1], str(frame))
                    self.assertEqual(args[args.index('--nav-config') + 1],
                                     str(scripts / 'nav_test/indoor_fly_nav.yaml'))

    def test_indoor6_scan_return_and_landing_progress(self):
        task = {"id": "test_indoor6", "automatic": True}
        health = {"ground": False, "armed": True, "mode": "OFFBOARD"}
        progress = board_tasks.Progress()
        progress.collect(['[12:00:00] test_indoor6 阶段：ROUTE',
                          '[12:00:01] INDOOR6 {"phase":"等高返回起飞点","route":{"points":[],"ids":[]}}'])
        self.assertIn("等高返回起飞点", progress.stage(task, health, True)[1])
        progress.collect(['[12:00:02] test_indoor6 阶段：LANDING'])
        self.assertIn("慢降", progress.stage(task, health, True)[1])
        self.assertIsNone(progress.scan)
        progress.collect(['[12:00:03] test_indoor6 阶段：DONE'])
        self.assertEqual(progress.stage(task, health, True)[0], "action_done")

    def test_damper_is_optional_and_survives_dispatch(self):
        import json
        from threading import Event
        from task_dispatch import Dispatch
        task = dict(id='test_indoor3_5', mapId='indoor', automatic=False)
        catalog = dict(tasks=[task],maps={'indoor':dict(key='shelf',sha256='test')})
        for extra in ({},{'damperDeg':20}):
            with tempfile.TemporaryDirectory() as tmp:
                cache = Path(tmp)
                (cache/'display').mkdir()
                (cache/'display/shelf.bin').touch()
                (cache/'display/shelf.json').write_text(json.dumps(dict(sha256='test')))
                q=Dispatch(catalog,cache,'orange',[],Event(),'~/catkin_ws')
                req=dict(id='a'*32,taskId=task['id'],mapKey='shelf',confirmAuto=False,**extra)
                q.submit(req)
                with patch.object(q,'remote',return_value=dict(boot='fixture',active=None,request=None)) as remote:
                    q.tick()
                payload=remote.call_args.args[1]
                self.assertEqual({k:v for k,v in payload.items() if k=='damperDeg'},extra)
                with patch.object(board_tasks,'catalog',return_value=catalog), patch.object(board_tasks,'BOOT','fixture'), \
                     patch.object(board_tasks,'get_record',return_value=None), patch.object(board_tasks,'active_record',return_value=None), \
                     patch.object(board_tasks,'session_exists',return_value=False), patch.object(board_tasks,'write'), \
                     patch.object(board_tasks,'tmux') as tmux, patch.object(board_tasks,'save') as save:
                    tmux.return_value.returncode=0
                    board_tasks.submit(payload)
                    args=board_tasks.script_command(task,save.call_args.args[0].get('damperDeg'))
                if extra:
                    self.assertEqual(args[-2:],['--damper-deg','20'])
                else:
                    self.assertNotIn('--damper-deg',args)
                for invalid in (True,-1,101,'20'):
                    with self.assertRaises(ValueError):
                        q.submit(dict(req,damperDeg=invalid))

    def test_task_logs_accumulate_without_overwriting(self):
        import json
        from threading import Event
        from task_dispatch import Dispatch

        progress = board_tasks.Progress()
        progress.collect([
            "[16:00:01] [ OK ] 定位就绪",
            "[16:00:02] 阶段：TAKEOFF",
            "[16:00:03] INDOOR6 {\"observation\":\"左下\",\"phase\":\"识别标签\",\"tagId\":5}",
        ])
        self.assertEqual([item["text"] for item in progress.logs], [
            "[16:00:01] [ OK ] 定位就绪",
            "[16:00:02] 阶段：TAKEOFF",
            "左下 · 识别标签 · Tag 5",
        ])

        task = dict(id="test_indoor3", label="test3", title="货架四点自动拍照", mapId="indoor", automatic=False)
        catalog = dict(tasks=[task], maps={"indoor": dict(key="shelf", sha256="test")})
        with tempfile.TemporaryDirectory() as tmp:
            cache = Path(tmp)
            (cache / "display").mkdir()
            (cache / "display/shelf.bin").touch()
            (cache / "display/shelf.json").write_text(json.dumps(dict(sha256="test")))
            q = Dispatch(catalog, cache, "orange", [], Event(), "~/catkin_ws")
            rid = "b" * 32
            q.submit(dict(id=rid, taskId="test_indoor3", mapKey="shelf", confirmAuto=False))
            q.control(dict(id=rid), "cancel")
            snap = q.snapshot(watch=False)
            texts = [item["text"] for item in snap["logs"]]
            self.assertEqual(len(texts), 2)
            self.assertIn("已确认任务", texts[0])
            self.assertEqual(texts[1], "已取消待下发任务")

    def test_legacy_board_scan_log_before_route_is_ready(self):
        from threading import Event
        from task_dispatch import Dispatch

        with tempfile.TemporaryDirectory() as tmp:
            dispatch = Dispatch({}, Path(tmp), "orange", [], Event())
            scan = dict(observation="左下", phase="识别标签", route=None)
            dispatch.board = dict(active=dict(id="b" * 32, state="running", scan=scan))
            dispatch._sync_board_logs(dispatch.board)
            scan["route"] = dict(ids=[5, 2])
            dispatch._sync_board_logs(dispatch.board)
            logs = dispatch.snapshot(watch=False)["logs"]
            self.assertEqual(len(logs), 2)
            self.assertNotIn("顺序", logs[0]["text"])
            self.assertIn("顺序 5 → 2", logs[1]["text"])
