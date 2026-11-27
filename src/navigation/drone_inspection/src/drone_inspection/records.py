"""Single-writer snapshots in the task's tmpfs directory; archived by bringup."""
import json
import os
from pathlib import Path
import time


def write(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False))
    os.replace(tmp, path)


class Records:
    def __init__(self, directory, run_id, mission, preview=False):
        self.directory = Path(directory)
        self.identity = dict(version=1, run_id=run_id, mission=mission['name'],
                             mode='preview' if preview else 'flight',
                             mission_sha256=mission['mission_sha256'])
        self.state = dict(self.identity, end='RUNNING', phase='BRINGUP')
        snapshot = self.directory / 'mission_state.json'
        if snapshot.is_file():
            previous = json.loads(snapshot.read_text())
            if any(previous.get(k) != v for k, v in self.identity.items()):
                raise ValueError('运行记录标识不匹配')
            self.state = previous
        else:
            self.scans({})
            self.update(phase='BRINGUP')

    def update(self, **change):
        if change.get('phase', self.state['phase']) != self.state['phase']:
            for key in ('action', 'waypoint_index', 'observation', 'tagId', 'route'):
                self.state.pop(key, None)
        self.state.update(change, time=time.time())
        write(self.directory / 'mission_state.json', self.state)
        line = json.dumps(self.state, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
        with (self.directory / 'mission_events.jsonl').open('a') as stream:
            stream.write(line + '\n')
        print('INSPECTION_EVENT ' + line, flush=True)

    def scans(self, record):
        write(self.directory / 'scan_results.json', dict(self.identity,
              batches=record.get('batches', []), scans=record.get('scans', []),
              tag_counts=record.get('tag_counts', {})))
