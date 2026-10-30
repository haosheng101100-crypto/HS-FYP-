"""COLLADA scene transforms and rotor sweep envelope of the unchanged Iris."""
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import json,hashlib,itertools,struct
ROOT=Path(__file__).resolve().parents[1];assets=ROOT/'work/iris_mesh_assets'
N={'c':'http://www.collada.org/2005/11/COLLADASchema'}

def vertices(path):
    root=E.parse(path).getroot();geometries={}
    for g in root.findall('c:library_geometries/c:geometry',N):
        mesh=g.find('c:mesh',N)
        if mesh is None:continue
        sources={s.attrib['id']:s for s in mesh.findall('c:source',N)};chunks=[]
        for v in mesh.findall('c:vertices',N):
            inp=v.find("c:input[@semantic='POSITION']",N);source=sources[inp.attrib['source'][1:]]
            a=np.fromstring(source.find('c:float_array',N).text,sep=' ')
            stride=int(source.find('c:technique_common/c:accessor',N).attrib.get('stride',3))
            chunks.append(a.reshape(-1,stride)[:,:3])
        if chunks:geometries[g.attrib['id']]=np.vstack(chunks)
    out=[]
    def walk(node,parent):
        M=parent.copy()
        for ch in node:
            tag=ch.tag.split('}')[-1]
            if tag=='matrix':M=M@np.fromstring(ch.text,sep=' ').reshape(4,4)
            elif tag in ('translate','rotate','scale'):raise ValueError('Implement explicit transform '+tag)
        for ch in node.findall('c:instance_geometry',N):
            v=geometries[ch.attrib['url'][1:]];out.append((np.c_[v,np.ones(len(v))]@M.T)[:,:3])
        for ch in node.findall('c:node',N):walk(ch,M)
    scene=root.find('c:library_visual_scenes/c:visual_scene',N)
    for node in scene.findall('c:node',N):walk(node,np.eye(4))
    result=np.vstack(out);unit=root.find('c:asset/c:unit',N)
    return result*(float(unit.attrib.get('meter',1)) if unit is not None else 1.)


def main():
    body=vertices(assets/'iris.dae');props={n:vertices(assets/n) for n in ('iris_prop_ccw.dae','iris_prop_cw.dae')}
    gimbal=[]
    for f in (ROOT/'work/gimbal_mesh_assets').glob('*.dae'):
        v=vertices(f)
        # Stage1 include pose: translation (0,-.01,-.124923), Rz(90) Rx(90).
        gimbal.append(v[:,[2,0,1]]+np.array([0.,-.01,-.124923]))
    gimbal=np.vstack(gimbal)
    stl=(assets/'iris_collision.stl').read_bytes();count=struct.unpack_from('<I',stl,80)[0]
    assert len(stl)==84+50*count
    collision=np.frombuffer(stl,dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]),offset=84)['vertices'].reshape(-1,3)
    sdf=E.parse(ROOT/'work/model_step04_v4/models/iris_with_standoffs_logged/model.sdf').getroot()
    lo=np.min([body.min(axis=0),gimbal.min(axis=0),collision.min(axis=0)],axis=0);hi=np.max([body.max(axis=0),gimbal.max(axis=0),collision.max(axis=0)],axis=0);rotors=[]
    for link in sdf.findall('.//link'):
        if not link.attrib['name'].startswith('rotor_'):continue
        origin=np.fromstring(link.find('pose').text,sep=' ')[:3]
        uri=link.find('visual/geometry/mesh/uri').text;mesh=props[uri.rsplit('/',1)[-1]]
        radius=float(np.linalg.norm(mesh[:,:2],axis=1).max());rotors.append(dict(origin=origin.tolist(),swept_radius=radius,half_height=float(abs(mesh[:,2]).max())))
        lo=np.minimum(lo,origin+[-radius,-radius,float(mesh[:,2].min())]);hi=np.maximum(hi,origin+[radius,radius,float(mesh[:,2].max())])
    report=dict(body_bounds=[body.min(axis=0).tolist(),body.max(axis=0).tolist()],gimbal_bounds=[gimbal.min(axis=0).tolist(),gimbal.max(axis=0).tolist()],rotors=rotors,
        iris_rotor_swept_bounds=[lo.tolist(),hi.tolist()],xy_bounding_rectangle_area_m2=float(np.prod((hi-lo)[:2])),
        scope='Body and fixed-gimbal visual meshes plus rotor swept disks, exact COLLADA transforms; existing legs lie inside XY bounds. Conservative horizontal bounding rectangle area, not nonconvex material silhouette area.',
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in assets.glob('*.dae')})
    out=ROOT/'outputs/receding_physical_20261023/iris_projection.json';out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    boxes=[(v.min(axis=0),v.max(axis=0)) for v in (body,gimbal,collision)]
    profile=json.loads((ROOT/'config/airframes/iris_fixed_gimbal_sim.json').read_text())
    for leg in profile['landing_legs']:
        c=np.array(leg['cylinder_center_body_m']);r=leg['radius_m'];h=leg['length_m']/2
        boxes.append((c-[r,r,h],c+[r,r,h]))
    points=[[float(x) for x in v] for low,high in boxes for v in itertools.product(*zip(low,high))]
    envelope=dict(simulation_only=True,fixed_points_body_m=points,rotors=rotors,
        bounding_rectangle_area_m2=report['xy_bounding_rectangle_area_m2'],
        area_convention='World-level axis-aligned occupancy rectangle containing body, fixed gimbal, landing legs and full rotor sweeps; not material silhouette area.',
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in list(assets.glob('*.dae'))+list((ROOT/'work/gimbal_mesh_assets').glob('*.dae'))+[assets/'iris_collision.stl',ROOT/'config/airframes/iris_fixed_gimbal_sim.json']})
    (ROOT/'config/airframes/iris_swept_envelope_v1.json').write_text(json.dumps(envelope,indent=2))


if __name__=='__main__':main()
