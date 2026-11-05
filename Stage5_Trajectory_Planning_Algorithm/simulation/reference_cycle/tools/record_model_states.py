from pathlib import Path
import json
import signal
import sys
import threading
import time
from google.protobuf.json_format import MessageToDict
from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts

output=Path(sys.argv[1]);output.mkdir(exist_ok=True)
stream=(output/'model_states.jsonl').open('x')
lock=threading.Lock();running=True;counts={};callbacks=[]
def stop(*args):
    global running
    running=False
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
node=Node()
from gz.msgs10.world_stats_pb2 import WorldStatistics
physics_paused=False
def on_stats(message):
    global physics_paused
    physics_paused=bool(message.paused)
node.subscribe(WorldStatistics,'/world/uav_vehicle_stage1/stats',on_stats)
def make_callback(kind):
    def callback(message):
        if physics_paused:return
        row={'kind':kind,'received_monotonic_s':time.monotonic(),
             'message':MessageToDict(message,preserving_proto_field_name=True)}
        with lock:
            stream.write(json.dumps(row,separators=(',',':'))+'\n')
            counts[kind]=counts.get(kind,0)+1
    return callback
for msgtype,topic,kind in [(Model,'/model/uav/rotor_joint_state','rotors'),
    (Model,'/model/uav/fixed_gimbal_joint_state','gimbal'),
    (Pose_V,'/model/uav/pose','uav_pose'),
    (Pose_V,'/model/landing_vehicle/pose','car_pose'),
    (Contacts,'/vehicle/deck/contact','deck_contact'),
    (Contacts,'/world/uav_vehicle_stage1/model/landing_vehicle/link/chassis/sensor/deck_contact/contact','deck_contact')]:
    callback=make_callback(kind);callbacks.append(callback)
    assert node.subscribe(msgtype,topic,callback)
while running:
    time.sleep(.5)
    with lock:stream.flush()
with lock:
    stream.flush()
    # Unsubscribe before interpreter destruction; callbacks are serialized by lock.
for topic in ['/model/uav/rotor_joint_state','/model/uav/fixed_gimbal_joint_state','/model/uav/pose','/model/landing_vehicle/pose','/vehicle/deck/contact','/world/uav_vehicle_stage1/model/landing_vehicle/link/chassis/sensor/deck_contact/contact']:
    node.unsubscribe(topic)
with lock:stream.close()
(output/'model_state_capture.json').write_text(json.dumps({'simulation_only':True,'counts':counts,'status':'closed'},indent=2))
print(json.dumps(counts),flush=True)
