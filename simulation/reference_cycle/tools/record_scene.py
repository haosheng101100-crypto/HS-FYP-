"""Encode the actual Gazebo observer-camera stream into a replayable MP4."""
import argparse
from pathlib import Path
import queue
import subprocess
import time
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

p=argparse.ArgumentParser()
p.add_argument('output');p.add_argument('--seconds',type=float,default=100)
a=p.parse_args();frames=queue.Queue(maxsize=12);n=Node()
def callback(msg):
    try:frames.put_nowait(msg)
    except queue.Full:pass
n.subscribe(Image,'/overview/image',callback)
first=frames.get(timeout=120)
out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
if out.exists():raise RuntimeError('Recording output already exists: '+str(out))
cmd=['ffmpeg','-loglevel','warning','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{first.width}x{first.height}',
     '-framerate','3','-i','pipe:0','-an','-c:v','libx264','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
start=first.header.stamp.sec+first.header.stamp.nsec*1e-9
stamp=start;count=0;wall_deadline=time.monotonic()+max(600,a.seconds*20)
try:
    msg=first
    while stamp-start<a.seconds and time.monotonic()<wall_deadline:
        if msg.step!=msg.width*3:raise RuntimeError('Expected RGB8 camera frame')
        proc.stdin.write(msg.data);count+=1
        if count%30==0:print(f'Recorded {stamp-start:.1f} simulation seconds, {count} camera frames',flush=True)
        msg=frames.get(timeout=120)
        stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
finally:
    proc.stdin.close();code=proc.wait(timeout=30)
    if code:raise RuntimeError(f'ffmpeg returned {code}')
print(f'Actual Gazebo video saved: {out}; frames={count}; sim seconds={stamp-start:.2f}',flush=True)
