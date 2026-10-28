"""Classic-only layout preview. Does not claim ArduPilot integration or flight."""
import copy
from pathlib import Path
import xml.etree.ElementTree as E

ROOT=Path(__file__).resolve().parents[1]
world=E.parse(ROOT/'worlds/uav_vehicle_stage1.sdf').getroot()
world.set('version','1.6');w=world.find('world');w.set('name','layout_preview_only')
for p in list(w.findall('plugin')):w.remove(p)
for inc in list(w.findall('include')):w.remove(inc)
w.find('physics').set('type','ode')
iris=copy.deepcopy(E.parse(ROOT/'vendor/ardupilot_gazebo/models/iris_with_standoffs/model.sdf').getroot().find('model'))
iris.set('name','uav_layout_preview');iris.find('pose').text='0 0 0.2 0 0 1.57079632679'
E.SubElement(iris,'static').text='true'
for parent in list(iris.iter()):
    for child in list(parent):
        if child.tag in ('plugin','sensor'):parent.remove(child)
for uri in iris.findall('.//uri'):
    if uri.text.startswith('model://'):
        uri.text='file://'+str(ROOT/'vendor/ardupilot_gazebo/models'/uri.text[len('model://'):])
w.append(iris)
vehicle=copy.deepcopy(E.parse(ROOT/'models/landing_vehicle/model.sdf').getroot().find('model'))
vehicle.find('static').text='true';E.SubElement(vehicle,'pose').text='5 0 0.002 0 0 0'
for p in list(vehicle.findall('plugin')):vehicle.remove(p)
w.append(vehicle)
gui=E.SubElement(w,'gui');camera=E.SubElement(gui,'camera',name='user_camera')
E.SubElement(camera,'pose').text='9 -10 7 0 0.5 2.1'
E.SubElement(camera,'view_controller').text='orbit'
E.indent(world)
E.ElementTree(world).write(ROOT/'worlds/layout_preview_only.world',encoding='utf-8',xml_declaration=True)
print('Layout preview generated; static models, no flight controller or cooperation active.')
