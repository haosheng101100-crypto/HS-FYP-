"""Diagnostic-only MAVLink capture. Never opens a connection or issues a command."""
from collections import Counter
import json,math

RECORDED_TYPES=frozenset({'ATTITUDE','LOCAL_POSITION_NED','ATTITUDE_TARGET','POSITION_TARGET_LOCAL_NED','NAV_CONTROLLER_OUTPUT','COMMAND_ACK','AUTOPILOT_VERSION','SYSTEM_TIME'})

def json_value(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {str(k):json_value(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [json_value(v) for v in value]
    if isinstance(value,bytes):return value.decode(errors='replace')
    return value

class FcuDiagnosticCapture:
    def __init__(self):self.rows=[];self.counts=Counter()
    def record(self,message,received_monotonic_s,latest_gazebo_sim_t_s):
        kind=message.get_type()
        if kind not in RECORDED_TYPES:return
        if not math.isfinite(received_monotonic_s):raise ValueError('Finite receipt time required')
        if latest_gazebo_sim_t_s is not None and not math.isfinite(latest_gazebo_sim_t_s):raise ValueError('Finite simulation receipt hint required')
        self.counts[kind]+=1
        self.rows.append({'received_monotonic_s':received_monotonic_s,'gazebo_sim_t_at_receipt_hint_s':latest_gazebo_sim_t_s,
                          'source_system':message.get_srcSystem(),'source_component':message.get_srcComponent(),'message':json_value(message.to_dict())})
    def save(self,directory):
        with (directory/'fcu_diagnostic_messages.jsonl').open('x') as stream:
            for row in self.rows:stream.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')
        summary={'simulation_only':True,'counts':dict(self.counts),'rows':len(self.rows),'missing_types':sorted(RECORDED_TYPES-set(self.counts)),
                 'requested_additional_message_ids':[83,85,62],'requested_additional_interval_us':50000,
                 'timing_interpretation':'Gazebo timestamp is a receipt-time hint, not the FCU sample time. Native message clocks are preserved inside each message. Missing streams are not reconstructed.'}
        (directory/'fcu_diagnostic_capture.json').write_text(json.dumps(summary,indent=2))
