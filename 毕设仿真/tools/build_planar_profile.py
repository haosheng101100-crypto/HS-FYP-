"""Build a separate XY guide fixture from the exact existing deck/chassis.

    python tools/build_planar_profile.py --source-model BASE/model.sdf --output NEW/model.sdf

Gazebo JointController force mode:
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointController.html
"""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import xml.etree.ElementTree as ET


def build_planar_model(source,destination):
    source=Path(source);destination=Path(destination)
    if source.resolve()==destination.resolve():raise ValueError('Do not overwrite the original physical model')
    original=source.read_bytes();root=ET.fromstring(original);model=root.find('model')
    if model is None or model.get('name')!='landing_vehicle':raise ValueError('Expected landing_vehicle model')
    chassis=model.find('link[@name="chassis"]')
    if chassis is None or chassis.find('collision[@name="landing_deck"]') is None:
        raise ValueError('Original deck/chassis/contact asset required')
    chassis=copy.deepcopy(chassis)
    pose_plugin=model.find('plugin[@name="gz::sim::systems::PosePublisher"]')
    if pose_plugin is None:raise ValueError('Original pose publisher required')
    pose_plugin=copy.deepcopy(pose_plugin)
    replacement=ET.Element('model',{'name':'landing_vehicle','canonical_link':'chassis'})
    ET.SubElement(replacement,'static').text='false'
    ET.SubElement(replacement,'self_collide').text='false'
    link=ET.SubElement(replacement,'link',{'name':'xy_carriage'})
    ET.SubElement(link,'gravity').text='false'
    inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text='2'
    inertia=ET.SubElement(inertial,'inertia')
    for key,value in dict(ixx=.1,iyy=.1,izz=.1,ixy=0,ixz=0,iyz=0).items():ET.SubElement(inertia,key).text=str(value)
    replacement.append(chassis)
    for axis,parent,child,vector in (('x','world','xy_carriage','1 0 0'),('y','xy_carriage','chassis','0 1 0')):
        joint=ET.SubElement(replacement,'joint',{'name':'planar_'+axis,'type':'prismatic'})
        ET.SubElement(joint,'parent').text=parent;ET.SubElement(joint,'child').text=child
        spec=ET.SubElement(joint,'axis');ET.SubElement(spec,'xyz',{'expressed_in':'__model__'}).text=vector
        limit=ET.SubElement(spec,'limit')
        for key,value in dict(lower=-100,upper=100,effort=60,velocity=1.).items():ET.SubElement(limit,key).text=str(value)
        plugin=ET.SubElement(replacement,'plugin',{'filename':'gz-sim-joint-controller-system','name':'gz::sim::systems::JointController'})
        for key,value in dict(joint_name='planar_'+axis,topic='/vehicle/planar_'+axis+'/cmd_vel',use_force_commands='true',p_gain=500,i_gain=0,d_gain=0,cmd_max=60,cmd_min=-60,initial_velocity=0).items():
            ET.SubElement(plugin,key).text=str(value)
    replacement.append(pose_plugin);root.remove(model);root.append(replacement)
    destination.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(root).write(destination,encoding='utf-8',xml_declaration=True)
    receipt=dict(source_sha256=hashlib.sha256(original).hexdigest(),generated_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
                 mechanism='fixed-yaw world-anchored XY prismatic guide; JointController force mode',
                 physical_validation='PENDING',original_chassis_preserved=True,
                 scope='Independent planar fixture, not the historical differential-drive vehicle')
    destination.with_suffix('.receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf-8')
    return receipt


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source-model',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();print(json.dumps(build_planar_model(args.source_model,args.output),indent=2))
