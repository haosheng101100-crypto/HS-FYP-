"""Read self-describing simulator DataFlash logs; no vehicle connection."""
from pathlib import Path
from collections import Counter
import argparse,hashlib,json,math
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
reader=mavutil.mavlink_connection(str(a.input));counts=Counter();parameters={};metadata={};messages=[]
types={'ATT','ANG','RATE','CTUN','PSCN','PSCE','PSCD','GUIP','SIM','SIM2','PIDN','PIDE','PIDA','PIDP','MODE','ARM','VER','MSG','DSF'}
def finite(value):
 if isinstance(value,float) and not math.isfinite(value):return None
 if isinstance(value,dict):return {str(k):finite(v) for k,v in value.items()}
 if isinstance(value,(list,tuple)):return [finite(v) for v in value]
 if isinstance(value,bytes):return value.decode(errors='replace')
 return value
while True:
 msg=reader.recv_match()
 if msg is None:break
 typ=msg.get_type();counts[typ]+=1
 if typ=='PARM':parameters[msg.Name]=msg.Value
 if typ in types:messages.append(finite(msg.to_dict()))
 if typ in ['FMT','FMTU','UNIT','MULT']:metadata.setdefault(typ,[]).append(finite(msg.to_dict()))
with (a.output/'control_messages.jsonl').open('x') as stream:
 for row in messages:stream.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')
for name,value in [('parameters.json',parameters),('format_metadata.json',metadata),('extraction_manifest.json',{'input':str(a.input.resolve()),'input_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest(),'counts':dict(counts),'selected_message_count':len(messages),'extractor_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'simulation_only':True})]:
 (a.output/name).write_text(json.dumps(finite(value),indent=2,allow_nan=False))
print(json.dumps({'input':str(a.input),'counts':dict(counts),'selected_message_count':len(messages)}))
