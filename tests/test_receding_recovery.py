from pathlib import Path
import sys,unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'work/receding_dependencies'))
import numpy as np
from landing.receding_live import braking_clearance_design,RecoveryReference
from landing.receding_landing import FlightState
from landing.execution import AircraftObservation
from landing.contact_geometry import Pose,RigidMotion


class RecoveryDesignTests(unittest.TestCase):
    def test_stopping_formula_matches_independent_time_integration(self):
        for speed,down_acc in ((.1,0.),(.3,.1),(.7,.2)):
            state=FlightState(0.,(0,0,1.),(0,0,-speed),(0,0,-down_acc))
            check=braking_clearance_design(state);dt=1e-5;t=np.arange(0.,3.,dt)
            a=np.where(t<.2,-down_acc,np.minimum(1.,-down_acc+2.*(t-.2)))
            v=-speed+np.cumsum(a)*dt;end=np.flatnonzero(v>=0)[0]
            loss=-np.trapezoid(v[:end],dx=dt)
            self.assertAlmostEqual(loss,check['stopping_loss_m'],delta=5e-5)

    def test_vertical_authority_retained_while_braking_horizontally(self):
        obs=AircraftObservation(0.,RigidMotion(Pose((2.,0.,1.3),(0,0,0,1)),(1.,0.,-.3),(0,0,0)),True,'GUIDED',(0,0,0))
        recovery=RecoveryReference(obs);previous=np.zeros(3)
        for i in range(80):
            now=.04*(i+1);ref,_=recovery.step(now,obs);a=np.array(ref.acceleration_world_m_s2)
            self.assertLessEqual(np.linalg.norm(a),1.2+1e-9)
            self.assertLessEqual(np.linalg.norm((a-previous)/.04),3.+1e-9)
            self.assertLessEqual(np.linalg.norm(a[:2]),.65+1e-9)
            self.assertLessEqual(abs(a[2]),1.+1e-9)
            obs=AircraftObservation(now,RigidMotion(Pose(ref.position_world_m,(0,0,0,1)),ref.velocity_world_m_s,(0,0,0)),True,'GUIDED',ref.acceleration_world_m_s2)
            previous=a


if __name__=='__main__':unittest.main()
