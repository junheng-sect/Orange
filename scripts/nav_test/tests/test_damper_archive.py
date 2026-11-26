"""Regression for early CH6 release, a second segment and a still taken in between."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from damper_archive import NS, supplement


class Segments(unittest.TestCase):
    def test_reopen(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            (root/'damper_deg.txt').write_text('20\n')
            photos=[]
            for off in (2,3,4):
                name=f'p{off}.png'
                (root/name).touch()
                photos.append(dict(file=name,status='saved',offset_s=off,stamp_ns=(1+off)*NS))
            goal=dict(end='SUCCEEDED',legs=[dict(index=1,sent_ros=.2,t0_ns=NS,confirmed_ns=2*NS,
                        score_end_ns=6*NS,photos=photos)])
            (root/'goal_test_indoor3_5_fixture.json').write_text(json.dumps(goal))
            bags=[]
            for num,(begin,end) in enumerate(((0,3.6),(4.6,7)),1):
                bag=root/f'test_indoor3_5_{num:06d}.bag'
                bag.touch();bags.append(bag)
                with bag.with_name(bag.stem+'_frames.csv').open('w') as f:
                    w=csv.writer(f);w.writerow(['source_stamp_ns','frame_index','source_seq'])
                    for i in range(round((end-begin)*15)+1):
                        w.writerow([round((begin+i/15)*NS),i,i])
                with bag.with_name(bag.stem+'_events.csv').open('w') as f:
                    w=csv.writer(f);w.writerow(['event','stamp_ns'])
                    w.writerows([('open',round(begin*NS)),('close',round(end*NS))])
            for i,bag in enumerate(bags):
                info,files=supplement(bag,lambda p:dict(name=p.name),[])
                point=info['routes'][0]['points'][0]
                self.assertEqual(info['hardness'],'20')
                self.assertTrue(point['steady_gaps'])
                self.assertEqual(bool(point['brake_gaps']),bool(i))
                own=point['photos'][2 if i else 0]
                self.assertIsNotNone(own['frame_index'])
                other=point['photos'][0 if i else 2]
                self.assertEqual(other['status'],'other_segment')
                self.assertNotIn(other['file'],{f['name'] for f in files})
                self.assertIsNone(point['photos'][1]['frame_index'])


if __name__=='__main__':
    unittest.main()
