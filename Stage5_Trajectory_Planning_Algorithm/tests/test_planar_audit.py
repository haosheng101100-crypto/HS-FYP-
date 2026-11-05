import copy
import unittest
from dataclasses import asdict
from landing.planar_landing import PlanarLandingSupervisor
from landing.receding_landing import FlightState,PlatformState
from landing.contact_geometry import Pose,RigidMotion
from landing.execution import AircraftObservation,BodyReference
from tools.audit_planar_policy import audit_rows


class PlanarReplayAuditTests(unittest.TestCase):
    def rows(self):
        policy=PlanarLandingSupervisor();rows=[]
        for i in range(31):
            t=i*.04;p=PlatformState(t,(.2*t,0,.8),(.2,0,0),(0,0,0))
            s=FlightState(t,(0,0,3),(0,0,0),(0,0,0))
            report=policy.update(t,s,p,2.,aligned=False,descending=False)
            report['inputs']=dict(aligned=False,descending=False,gap_m=2.,contact=False)
            rows.append(dict(sim_t_s=t,platform=asdict(p),com_state=asdict(s),
                measured_leg_gap_m=2.,planar_supervision=report,decision={'phase':'ALIGNING'},forecast_uncertainty=None))
        return rows

    def test_reconstructs_causal_decisions_from_saved_states(self):
        result=audit_rows(self.rows(),{})
        self.assertTrue(result['passed'],result)
        self.assertFalse(result['physics_validated'])

    def test_forged_descent_during_unstable_motion_is_rejected(self):
        rows=self.rows();rows[0]['planar_supervision']['action']='DESCEND'
        self.assertFalse(audit_rows(rows,{})['passed'])

    def test_forged_or_missing_policy_inputs_fail_closed(self):
        for mutation in ('gap','missing'):
            rows=self.rows()
            if mutation=='gap':rows[2]['planar_supervision']['inputs']['gap_m']=99.
            else:rows[2]['planar_supervision'].pop('inputs')
            self.assertFalse(audit_rows(rows,{})['passed'])

    def test_contact_claim_needs_physical_or_native_handover_evidence(self):
        rows=self.rows()[:1];rows[0]['planar_supervision']={'action':'CONTACT','reason':'PHYSICAL_CONTACT_OWNS_CONTROL',
            'inputs':dict(aligned=False,descending=False,gap_m=2.,contact=True)}
        self.assertFalse(audit_rows(rows,{})['passed'])

    def test_alignment_complete_needs_a_real_ready_window(self):
        rows=self.rows()[:1]
        events=[{'time':0.,'event':'ALIGNMENT_COMPLETE'}]
        self.assertFalse(audit_rows(rows,{},events=events)['passed'])

    def test_native_contact_retains_last_sent_reference_across_null_outputs(self):
        rows=self.rows()
        reference=asdict(BodyReference((0,0,3),(0,0,0),(0,0,0),0.))
        rows[-1]['sent_decision']={'reference':reference}
        for i in (31,32):
            t=i*.04;p=PlatformState(t,(.2*t,0,.8),(.2,0,0),(0,0,0))
            report={'action':'CONTACT','reason':'PHYSICAL_CONTACT_OWNS_CONTROL',
                'inputs':dict(aligned=False,descending=False,gap_m=0.,contact=True,braking_reference=reference)}
            rows.append(dict(sim_t_s=t,platform=asdict(p),com_state=asdict(FlightState(t,(0,0,1),(0,0,0),(0,0,0))),
                measured_leg_gap_m=0.,planar_supervision=report,decision={'phase':'CONTACT_HANDOVER'},
                sent_decision={'reference':None},contact_frame={'active':i==31},forecast_uncertainty=None))
        result=audit_rows(rows,{})
        self.assertTrue(result['passed'],result['failures'])


if __name__=='__main__':unittest.main()
