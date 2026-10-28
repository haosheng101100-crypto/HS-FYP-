"""Stream actual Gazebo camera PNGs and latest controller status over SSH stdout."""
import json,queue,struct,sys,zlib
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

root=Path(__file__).resolve().parents[1]
frames=queue.Queue(maxsize=2)
def receive(msg):
    try:frames.put_nowait(msg)
    except queue.Full:pass
def chunk(kind,data):
    return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
n=Node();n.subscribe(Image,'/overview/image',receive)
while True:
    msg=frames.get(timeout=90)
    if msg.step!=msg.width*3:raise RuntimeError('Camera must be RGB8')
    scan=b''.join(b'\x00'+msg.data[y*msg.step:(y+1)*msg.step] for y in range(msg.height))
    png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',msg.width,msg.height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(scan,1))+chunk(b'IEND',b'')
    state={'camera_sim_t':msg.header.stamp.sec+msg.header.stamp.nsec*1e-9}
    p=root/'results/progress.json'
    try:
        d=json.loads(p.read_text())
        for key in ['phase','armed','observed_xy_error','observed_car_speed','observed_foot_height']:
            state[key]=d.get(key)
    except (FileNotFoundError,json.JSONDecodeError):pass
    metadata=json.dumps(state,allow_nan=False).encode()
    sys.stdout.buffer.write(struct.pack('!II',len(png),len(metadata))+png+metadata)
    sys.stdout.buffer.flush()
