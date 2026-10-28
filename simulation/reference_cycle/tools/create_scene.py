"""Extend the official Iris runway world with a wheel-driven landing vehicle."""
from pathlib import Path
from copy import deepcopy
import xml.etree.ElementTree as E

ROOT = Path(__file__).resolve().parents[1]

def node(parent, tag, value=None, **attrs):
    e = E.SubElement(parent, tag, attrs)
    if value is not None: e.text = str(value)
    return e

def plugin(parent, filename, name):
    return node(parent, 'plugin', filename=filename, name=name)

def inertial(link, mass, ix, iy, iz):
    i = node(link, 'inertial'); node(i, 'mass', mass); tensor = node(i, 'inertia')
    for key, value in [('ixx',ix),('iyy',iy),('izz',iz),('ixy',0),('ixz',0),('iyz',0)]:
        node(tensor, key, value)

def shape(link, name, kind, values, color, pose=None, friction=1):
    for tag in ['collision','visual']:
        part = node(link, tag, name=name)
        if pose: node(part, 'pose', pose)
        geom = node(part, 'geometry'); specific = node(geom, kind)
        for key,value in values.items(): node(specific,key,value)
        if tag == 'visual':
            mat=node(part,'material');node(mat,'ambient',color);node(mat,'diffuse',color)
        else:
            ode=node(node(node(part,'surface'),'friction'),'ode')
            node(ode,'mu',friction);node(ode,'mu2',friction)

def pose_publisher(model):
    p=plugin(model,'gz-sim-pose-publisher-system','gz::sim::systems::PosePublisher')
    for key,value in [('publish_link_pose','false'),('publish_sensor_pose','false'),('publish_collision_pose','false'),('publish_visual_pose','false'),('publish_nested_model_pose','false'),('publish_model_pose','true'),('use_pose_vector_msg','true'),('update_frequency',30)]:
        node(p,key,value)

def write(root,path):
    E.indent(root);path.parent.mkdir(parents=True,exist_ok=True)
    E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

def main():
    sdf=E.Element('sdf',version='1.9');car=node(sdf,'model',name='landing_vehicle')
    node(car,'static','false');node(car,'self_collide','false')
    body=node(car,'link',name='chassis');node(body,'pose','0 0 0.45 0 0 0')
    inertial(body,50,9,19,25)
    shape(body,'body','box',{'size':'2 1.4 0.3'},'0.08 0.2 0.55 1')
    # Deck top is z=0.80 m in the vehicle model frame, with real collision.
    shape(body,'landing_deck','box',{'size':'2 1.4 0.12'},'0.95 0.65 0.1 1','0 0 0.29 0 0 0')
    contact=node(body,'sensor',name='deck_contact',type='contact')
    node(contact,'always_on','true');node(contact,'update_rate',30)
    node(contact,'topic','/vehicle/deck/contact')
    node(node(contact,'contact'),'collision','landing_deck')
    # An observer camera travels with the chassis for a readable scene recording.
    # It is a sensor only, with no extra mass or collision geometry.
    observer=deepcopy(E.parse(ROOT/'models/observer_camera.sdf').find('model/link/sensor'))
    observer.find('pose').text='-9 -7 4.55 0 0.42 0.82'
    body.append(observer)
    # Markings are visual only; they do not introduce additional contact surfaces.
    for name,size in [('mark_x','0.75 0.04 0.002'),('mark_y','0.04 0.75 0.002')]:
        part=node(body,'visual',name=name);node(part,'pose','0 0 0.351 0 0 0')
        node(node(node(part,'geometry'),'box'),'size',size)
        mat=node(part,'material');node(mat,'diffuse','1 1 1 1');node(mat,'ambient','1 1 1 1')
    for side,y in [('left',0.78),('right',-0.78)]:
        wheel=node(car,'link',name=side+'_wheel');node(wheel,'pose',f'0 {y} 0.25 0 0 0')
        inertial(wheel,2,0.035,0.0625,0.035)
        shape(wheel,'wheel','cylinder',{'radius':0.25,'length':0.12},'0.06 0.06 0.06 1','0 0 0 1.57079632679 0 0')
        j=node(car,'joint',name=side+'_wheel_joint',type='revolute')
        node(j,'parent','chassis');node(j,'child',side+'_wheel')
        axis=node(j,'axis');node(axis,'xyz','0 1 0');limit=node(axis,'limit')
        node(limit,'lower',-1e16);node(limit,'upper',1e16)
    for name,x in [('front',0.8),('rear',-0.8)]:
        caster=node(car,'link',name=name+'_caster');node(caster,'pose',f'{x} 0 0.08 0 0 0')
        inertial(caster,0.2,0.001,0.001,0.001)
        shape(caster,'caster','sphere',{'radius':0.08},'0.1 0.1 0.1 1',friction=0.001)
        j=node(car,'joint',name=name+'_caster_joint',type='fixed')
        node(j,'parent','chassis');node(j,'child',name+'_caster')
    d=plugin(car,'gz-sim-diff-drive-system','gz::sim::systems::DiffDrive')
    for key,value in [('left_joint','left_wheel_joint'),('right_joint','right_wheel_joint'),('wheel_separation',1.56),('wheel_radius',0.25),('topic','/vehicle/cmd_vel'),('odom_topic','/vehicle/odometry'),('odom_publish_frequency',30),('max_linear_velocity',1),('min_linear_velocity',-1),('max_linear_acceleration',0.3),('min_linear_acceleration',-0.3)]:node(d,key,value)
    pose_publisher(car)
    write(sdf,ROOT/'models/landing_vehicle/model.sdf')
    config=E.Element('model');node(config,'name','Landing vehicle');node(config,'version','1.0')
    node(config,'sdf','model.sdf',version='1.9');node(config,'description','Wheel-driven flat-ground platform; deck top 0.80 m.')
    write(config,ROOT/'models/landing_vehicle/model.config')

    upstream=ROOT/'vendor/ardupilot_gazebo'
    sdf=E.parse(upstream/'worlds/iris_runway.sdf').getroot()
    w=sdf.find('world');w.set('name','uav_vehicle_stage1')
    # Keep the original terrain, lighting, coordinate conversion and flight model.
    plugin(w,'gz-sim-contact-system','gz::sim::systems::Contact')
    iris=E.parse(upstream/'models/iris_with_gimbal/model.sdf').getroot()
    model=iris.find('model');model.set('name','stage1_iris');pose_publisher(model)
    write(iris,ROOT/'models/stage1_iris/model.sdf')
    cfg=E.Element('model');node(cfg,'name','Official Iris with pose output');node(cfg,'version','1.0');node(cfg,'sdf','model.sdf',version='1.9')
    write(cfg,ROOT/'models/stage1_iris/model.config')
    include=next(i for i in w.findall('include') if i.findtext('uri')=='model://iris_with_gimbal')
    include.find('uri').text='model://stage1_iris';node(include,'name','uav')
    include=node(w,'include');node(include,'uri','model://landing_vehicle');node(include,'pose','5 0 0.002 0 0 0')
    write(sdf,ROOT/'worlds/uav_vehicle_stage1.sdf')
    print('Extended official iris_runway.sdf with landing vehicle:',ROOT)

if __name__=='__main__':main()
