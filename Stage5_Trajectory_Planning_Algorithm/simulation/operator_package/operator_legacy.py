"""Human-controlled phases around the existing simulated landing controller."""
from pathlib import Path
import os,json,time,sys,math
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from interactive_base import LiveApproach
from operator_clock import allowed_actions
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from pymavlink import mavutil
from landing.mavlink_reference import LocalNedAlignment
from gz.msgs10.twist_pb2 import Twist

SESSION=Path(os.environ['HS_OPERATOR_SESSION'])

class OperatorApproach(LiveApproach):
    def __init__(self):
        self.operator_stage='INITIALIZING';self.operator_message='飞控初始化中，尚未起飞'
        self.takeoff_requested=False;self.land_requested=False;self.reset_requested=False
        self.close_requested=False;self.last_status=-math.inf
        super().__init__()
        self.node.subscribe(WorldStatistics,'/world/uav_vehicle_stage1/stats',self.world_stats)
        self.result['operator_controlled']=True
        self.result['watchdog_wall_clock']='Active monotonic time excluding confirmed deliberate Gazebo pauses. Simulation-age gates remain unchanged.'
    def world_stats(self,msg):self.operator_clock.set_paused(bool(msg.paused))
    def pause_world(self,pause):
        if not pause:self.operator_clock.set_paused(False)
        ok,response=self.node.request('/world/uav_vehicle_stage1/control',WorldControl(pause=pause),WorldControl,Boolean,5000)
        if not ok or not response.data:raise RuntimeError('Gazebo pause/resume request failed')
        self.operator_clock.set_paused(pause)
        self.event('Operator physics '+('PAUSE' if pause else 'RESUME'))
    def process_actions(self):
        request_dir=SESSION/'requests';archive=SESSION/'processed';archive.mkdir(exist_ok=True)
        for p in sorted(request_dir.glob('*.json')):
            req=json.loads(p.read_text());action=req['action'];allowed=allowed_actions(self.operator_stage,self.armed(),self.operator_clock.paused)
            if action=='close':self.close_requested=True
            elif not allowed.get(action,False):self.operator_message='当前阶段不能执行这个操作'
            elif action=='takeoff':self.takeoff_requested=True;self.operator_message='收到起飞指令'
            elif action=='land':self.land_requested=True;self.operator_message='收到降落到小车指令'
            elif action=='pause':self.pause_world(True);self.operator_message='已暂停：物理时间和任务计时均冻结'
            elif action=='resume':self.pause_world(False);self.operator_message='已继续'
            elif action=='reset':self.reset_requested=True;self.running=False
            p.rename(archive/(ROOT.name+'__'+p.name))
    def publish_status(self,force=False):
        now=time.monotonic()
        if not force and now-self.last_status<.2:return
        self.last_status=now
        status={'stage':self.operator_stage,'message':self.operator_message,'armed':self.armed(),
            'paused':self.operator_clock.paused,'sim_t_s':self.sim_t(),'body_height_m':self.latest.pose.position_world_m[2] if self.latest else None,
            'buttons':allowed_actions(self.operator_stage,self.armed(),self.operator_clock.paused),
            'cycle':ROOT.name,'run':str(ROOT),'real_wall_unix_s':time.time(),'manual_takeoff_required':True,'manual_landing_required':True}
        tmp=SESSION/'status.tmp';tmp.write_text(json.dumps(status,ensure_ascii=False));tmp.replace(SESSION/'status.json')
    def pump(self):
        self.process_actions()
        active=self.enabled;was_paused=self.operator_clock.paused
        if was_paused:self.enabled=False
        try:super().pump()
        finally:
            if was_paused:self.enabled=active
        self.publish_status()
        if self.close_requested:raise RuntimeError('Operator closed control panel')
    def run(self):
        self.wait(lambda:'HEARTBEAT' in self.telemetry,60,'heartbeat')
        self.m.mav.param_set_send(1,1,b'GUID_OPTIONS',32.,mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self.wait(lambda:'PARAM_VALUE' in self.telemetry and self.telemetry['PARAM_VALUE'][0].param_id=='GUID_OPTIONS' and self.telemetry['PARAM_VALUE'][0].param_value==32.,30,'GUID_OPTIONS acknowledgement')
        self.result['horizontal_control']={'native_guided_options_confirmed':32,'position_gain':.4,'velocity_gain':1.,'acceleration_lead_s':.25,'maximum_acceleration_m_s2':2.5}
        for mid in (30,32,33,245,83,85,62):self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,mid,50000)
        self.wait(lambda:self.latest and self.car_history and 'LOCAL_POSITION_NED' in self.telemetry,180,'estimator initialization')
        t=self.sim_t();self.wait(lambda:self.sim_t()-t>=12.,180,'sensor settling')
        self.m.set_mode(self.m.mode_mapping()['GUIDED']);self.wait(lambda:self.mode()=='GUIDED',30,'GUIDED')
        self.operator_stage='READY';self.stage='WAIT_USER_TAKEOFF';self.operator_message='就绪：等待你点击“起飞”'
        self.pause_world(True);self.publish_status(True)
        while self.running and not self.takeoff_requested:self.pump()
        if self.reset_requested:return
        self.pause_world(False);self.operator_stage='TAKING_OFF';self.operator_message='正在起飞，随后只悬停，等待你的降落指令'
        self.stage='ARM'
        for attempt in range(8):
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1)
            try:self.wait(self.armed,12,'normal arming');break
            except RuntimeError:
                if attempt==7:raise
        local=self.telemetry['LOCAL_POSITION_NED'][0]
        self.alignment=LocalNedAlignment.from_body_measurement(self.latest.pose.position_world_m,(local.x,local.y,local.z))
        self.stage='TAKEOFF';self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,4.)
        self.wait(lambda:self.latest.pose.position_world_m[2]>3.7,180,'physical takeoff')
        self.stage='USER_HOVER';self.enabled=True;self.prepare_start=self.sim_t()
        self.operator_stage='HOVER';self.operator_message='悬停中：只有你点击“降落到小车”才开始着陆'
        while self.running and not self.land_requested:self.pump()
        self.stage='PREPARE';self.prepare_start=self.sim_t();self.prepare_ready=None
        self.operator_stage='LANDING';self.operator_message='执行你指定的降落：小车匀速前进，下降中制动'
        self.wait(lambda:self.executor.experiment_can_finish(self.sim_t(),self.armed(),self.disarmed_since),600,'landing and support')
        self.result['failures']=list(self.executor.handover.failures);self.result['normal_disarm_observed']=not self.armed()
        self.result['main_approach_start_sim_t_s']=self.executor.start_s
        self.enabled=False;self.car_speed=0.;self.car_pub.publish(Twist())
        self.operator_stage='COMPLETE' if self.result['completed'] else 'ERROR'
        self.operator_message='已落台并正常停转。点击“复位”可重新开始' if self.result['completed'] else '本轮未通过；已正常退出，可复位'
        self.publish_status(True)
        while self.running and not self.reset_requested:self.pump()

if __name__=='__main__':
    demo=OperatorApproach();code=0
    try:
        demo.run()
        if demo.reset_requested:code=10
    except Exception as exc:
        if demo.operator_clock.paused:demo.pause_world(False)
        demo.emergency(exc);demo.operator_stage='ERROR';demo.operator_message='已退出：'+str(exc);demo.publish_status(True);code=2
    finally:demo.save()
    sys.exit(code)
