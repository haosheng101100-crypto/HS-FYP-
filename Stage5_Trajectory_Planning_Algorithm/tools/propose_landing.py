"""Offline evidence-lookup demo. Does not connect to Gazebo or a flight controller."""
from pathlib import Path
import json,argparse,sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from landing.decision_baseline import propose_from_evidence

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--query',type=Path,required=True)
    p.add_argument('--dataset',type=Path,default=ROOT/'outputs/model_audit/step15/decision_baseline_v2/flight_records.jsonl')
    p.add_argument('--output',type=Path);a=p.parse_args()
    query=json.loads(a.query.read_text(encoding='utf-8'));records=[json.loads(l) for l in a.dataset.read_text(encoding='utf-8').splitlines()]
    cfg=json.loads((ROOT/'config/experiments/decision_baseline_v1.json').read_text())
    result=propose_from_evidence(query['features'],records,cfg['context_match'],query['maximum_target_closing_speed_m_s'])
    text=json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)
    if a.output:
        if a.output.exists():raise ValueError('Preserve existing proposal output')
        a.output.write_text(text,encoding='utf-8')
    print(text)
