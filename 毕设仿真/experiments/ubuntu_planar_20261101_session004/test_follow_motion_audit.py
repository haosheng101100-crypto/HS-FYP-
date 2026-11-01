import unittest,json,tempfile,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'repository/毕设仿真'))
from tools import audit_guided_motion as old
from tools import audit_planar_guided_motion as new
class FollowCoverageTests(unittest.TestCase):
 def test_pure_follow_recovery_is_not_excluded(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);(root/'results').mkdir();(root/'config/experiments').mkdir(parents=True)
   (root/'config/airframes').mkdir()
   (root/'results/receding_control.jsonl').write_text(json.dumps({'sim_t_s':1.,'planner_phase':'PLANAR_FOLLOW'})+'\n'+json.dumps({'sim_t_s':1.1,'planner_phase':'RECOVERY_REQUIRED'})+'\n')
   (root/'config/experiments/receding_live_v1.json').write_text(json.dumps({'planner_limits':{'speed':3.,'acceleration':1.5}}))
   (root/'config/experiments/execution_v1.json').write_text(json.dumps({'airframe_profile':'config/airframes/test.json'}));(root/'config/airframes/test.json').write_text(json.dumps({'nominal_com_in_body_m':[0,0,0]}))
   (root/'results/execution_status.json').write_text(json.dumps({'completed':False,'scenario':'planar_xy'}))
   lines=[]
   for i in range(11):
    t=1.+.01*i;lines.append(json.dumps({'kind':'uav_pose','message':{'pose':[{'name':'uav','header':{'stamp':{'sec':1,'nsec':10000000*i}},'position':{'x':.2*(t-1),'y':0,'z':2.},'orientation':{'w':1}}]}}))
   raw=('\n'.join(lines)+'\n').encode();(root/'results/model_states.jsonl').write_bytes(raw)
   import hashlib
   (root/'results/raw_compression.json').write_text(json.dumps({'sha256_uncompressed':hashlib.sha256(raw).hexdigest()}))
   with self.assertRaisesRegex(AssertionError,'No optimized'):old.audit(root)
   report=new.audit(root);self.assertTrue(report['passed']);self.assertEqual(report['declared_interval_s'],[1.,1.1]);self.assertAlmostEqual(report['peak_speed_m_s'],.2)
if __name__=='__main__':unittest.main()
