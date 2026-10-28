"""Capture actual new numerical plans for an honest interactive replay.

No simulator or actuator connection. Source JSONs remain unchanged.
"""
from pathlib import Path
import sys
import json
import re
import hashlib
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_receding_closed_loop as simulation
from landing.receding_landing import RecedingLandingSession
from datetime import datetime, timezone

captured=[]


class RecordingSession(RecedingLandingSession):
    def update(self,*args,**kwargs):
        result=super().update(*args,**kwargs)
        plan=result.get('reference')
        if plan is not None and result.get('report') is not None:
            path=[]
            for i in range(101):
                elapsed=plan.duration*i/100
                path.append([plan.start+elapsed,*plan.at(elapsed)['position'].tolist()])
            captured.append({'t':plan.start,'path':path})
        return result


if __name__=='__main__':
    simulation.RecedingLandingSession=RecordingSession
    folder=simulation.ROOT/'outputs/receding_landing_20261023'/('visual_replay_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    folder.mkdir(parents=True,exist_ok=False)
    data={}
    for key in ['constant_velocity','speed_change','turning']:
        captured.clear();result=simulation.run(key)
        rows=[[r['time_s'],*r['com_position'],*r['platform_position']] for r in result['rows']]
        data[key]={'rows':[[round(v,6) for v in r] for r in rows],
            'plans':[{'t':p['t'],'path':[[round(v,6) for v in r] for r in p['path']]} for p in captured],
            'initial':[0.,0.,2.],'deckInitial':[.5,0.,.8]}
        result['captured_plans']=data[key]['plans']
        (folder/(key+'.json')).write_text(json.dumps(result,ensure_ascii=False),encoding='utf-8')
        print(key,result['outcome'],'captured_plans',len(captured),flush=True)
    target=Path('C:/Users/new/.codex/visualizations/2026/10/23/01a16731-18fc-7462-9cb5-0c77212f612d/landing-motion.html')
    text=target.read_text(encoding='utf-8')
    text=re.sub(r'(<script type="application/json" id="lm-data">).*?(</script>)',
        lambda m:m[1]+json.dumps(data,separators=(',',':'))+m[2],text,flags=re.S)
    target.write_text(text,encoding='utf-8')
    (folder/'manifest.json').write_text(json.dumps({'scope':'Lagged point-mass numerical replay, no contact dynamics or FCU',
        'fragment':str(target),'fragment_sha256':hashlib.sha256(target.read_bytes()).hexdigest()},indent=2),encoding='utf-8')
    print(folder, target.stat().st_size,flush=True)
