import unittest
from copy import deepcopy
from tools.audit_planar_policy import audit_rows

class ContactPolicyEvidenceTests(unittest.TestCase):
 def row(self, frame):
  return {'sim_t_s':5.,'platform':{'stamp':5.,'position':[0,0,.8],'velocity':[0,0,0],'acceleration':[0,0,0]},'com_state':{'stamp':5.,'position':[0,0,1.],'velocity':[0,0,0],'acceleration':[0,0,0]},'measured_leg_gap_m':0.,'planar_supervision':{'action':'CONTACT','reason':'PHYSICAL_CONTACT_OWNS_CONTROL','inputs':{'aligned':False,'descending':False,'gap_m':0.,'contact':True}},'contact_frame':frame,'decision':{'phase':'CONTACT_HANDOVER'}}
 def test_serialized_contact_frame_without_active_is_physical_evidence(self):
  row=self.row({'sim_t_s':5.,'leg_names':['rear_left_leg_collision'],'unexpected_uav_collisions':[]})
  self.assertTrue(audit_rows([row],{},events=[])['passed'])
 def test_first_contact_event_before_native_capture_is_evidence(self):
  row=self.row(None);row['first_new_contact_event']={'sim_t_s':4.98,'leg_names':['rear_left_leg_collision'],'unexpected_uav_collisions':[]}
  self.assertTrue(audit_rows([row],{},events=[])['passed'])
 def test_empty_frame_cannot_authorize_contact(self):
  self.assertFalse(audit_rows([self.row({'sim_t_s':5.,'leg_names':[],'unexpected_uav_collisions':[]})],{},events=[])['passed'])
 def test_future_or_stale_frame_cannot_authorize_contact(self):
  for stamp in [5.1,4.5]:
   with self.subTest(stamp=stamp):self.assertFalse(audit_rows([self.row({'sim_t_s':stamp,'leg_names':['rear_left_leg_collision'],'unexpected_uav_collisions':[]})],{},events=[])['passed'])
 def test_latched_contact_remains_owned_after_frame_clears(self):
  first=self.row({'sim_t_s':5.,'leg_names':['rear_left_leg_collision'],'unexpected_uav_collisions':[]});second=deepcopy(first);second['sim_t_s']=5.04;second['platform']['stamp']=5.04;second['com_state']['stamp']=5.04;second['contact_frame']=None
  self.assertTrue(audit_rows([first,second],{},events=[])['passed'])
