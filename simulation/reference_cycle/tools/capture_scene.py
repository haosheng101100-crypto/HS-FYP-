"""Save a genuine Gazebo camera frame as PNG, without a separate renderer."""
import argparse
from pathlib import Path
import struct
import threading
import zlib
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

p=argparse.ArgumentParser()
p.add_argument('output')
p.add_argument('--topic',default='/overview/image')
a=p.parse_args()
ready=threading.Event(); frames=[]
def callback(msg):
    if msg.width and msg.height:
        frames.append(msg)
        if len(frames)>=4:ready.set()
n=Node()
if not n.subscribe(Image,a.topic,callback):raise RuntimeError('Camera subscription failed')
if not ready.wait(30):raise RuntimeError('No Gazebo camera frames received')
msg=frames[-1]
if msg.step != msg.width*3:raise RuntimeError(f'Expected RGB8; got format {msg.pixel_format_type}, stride {msg.step}')
def chunk(kind,data):
    return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
scan=b''.join(b'\x00'+msg.data[y*msg.step:(y+1)*msg.step] for y in range(msg.height))
png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',msg.width,msg.height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(scan))+chunk(b'IEND',b'')
out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(png)
print(f'Gazebo frame saved: {out} ({msg.width}x{msg.height}), sim time {msg.header.stamp.sec}.{msg.header.stamp.nsec:09d}')
