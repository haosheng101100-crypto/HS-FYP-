"""Wait for Gazebo sensor initialization before starting the JSON flight model."""
import argparse
import threading
from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.clock_pb2 import Clock

p=argparse.ArgumentParser();p.add_argument('--world',required=True);a=p.parse_args()
n=Node();ready=threading.Event();state={'clock':False,'imu':False}
def clock(m):
    state['clock']=m.sim.sec+m.sim.nsec*1e-9>2
    if all(state.values()):ready.set()
def imu(m):
    accel=m.linear_acceleration
    state['imu']=accel.x**2+accel.y**2+accel.z**2>1
    if all(state.values()):ready.set()
n.subscribe(Clock,'/world/'+a.world+'/clock',clock)
name='iris_with_gimbal' if a.world=='iris_runway' else 'uav'
n.subscribe(IMU,f'/world/{a.world}/model/{name}/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu',imu)
if not ready.wait(180):raise RuntimeError('Gazebo IMU initialization did not finish')
print('Gazebo clock and IMU initialized; starting ArduCopter')
