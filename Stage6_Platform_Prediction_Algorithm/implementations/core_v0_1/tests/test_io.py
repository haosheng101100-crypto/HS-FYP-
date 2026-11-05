import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from stage6.io import predict_file


def line(t,x=None):
    return json.dumps(dict(stamp=t,received_at=t,position_xy=[t if x is None else x,0.],yaw=0.,covariance=[[.0025,0,0],[0,.0025,0],[0,0,.0004]]))


class IOTests(unittest.TestCase):
    def test_causal_file_replay_and_configuration_saved(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'poses.jsonl';target=root/'pred.jsonl'
            source.write_text('\n'.join(line(i/10) for i in range(11)))
            result=predict_file(source,target,particles_per_mode=2)
            rows=[json.loads(r) for r in target.read_text().splitlines()]
            self.assertEqual(result['observations'],11)
            self.assertEqual(len(rows),6)
            self.assertEqual(rows[0]['status'],'warming_up')
            self.assertEqual(len(rows[-1]['models']),4)
            self.assertEqual(rows[-1]['configuration']['state_order'],['x','y','psi','v','a','kappa','j','eta'])
            self.assertEqual(rows[-1]['configuration']['observation_offset_xy'],[0.,0.])

    def test_future_suffix_cannot_change_saved_prefix(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for name,changed in [('a',False),('b',True)]:
                p=root/(name+'.jsonl')
                p.write_text('\n'.join(line(i/10,i/10+.03 if changed and i>7 else None) for i in range(11)))
                predict_file(p,root/(name+'_out.jsonl'),particles_per_mode=2)
            def prefix(name):
                return [r for r in map(json.loads,(root/(name+'_out.jsonl')).read_text().splitlines()) if r['issued_at']<=.6]
            self.assertEqual(prefix('a'),prefix('b'))

    def test_invalid_input_cleans_output_and_existing_output_untouched(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'bad.jsonl';target=root/'out.jsonl'
            source.write_text(line(0)+'\n'+json.dumps(dict(stamp=1,received_at=0)))
            with self.assertRaises(ValueError):predict_file(source,target)
            self.assertFalse(target.exists())
            target.write_text('existing')
            with self.assertRaises(FileExistsError):predict_file(source,target)
            self.assertEqual(target.read_text(),'existing')

    def test_unknown_fields_and_empty_file_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'poses.jsonl';target=root/'out.jsonl'
            for content in ['',json.dumps(dict(json.loads(line(0)),future_command=1))]:
                source.write_text(content)
                with self.assertRaises(ValueError):predict_file(source,target)
                self.assertFalse(target.exists())

    def test_command_line_runs_and_invalid_arguments_fail(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'poses.jsonl';source.write_text(line(0))
            result=subprocess.run([sys.executable,'-m','stage6','--input',str(source),'--output',str(root/'out.jsonl')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout)['observations'],1)
            result=subprocess.run([sys.executable,'-m','stage6','--input',str(source),'--output',str(root/'bad.jsonl'),'--every','0'],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertFalse((root/'bad.jsonl').exists())
