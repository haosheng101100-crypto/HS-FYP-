"""Generate auditable reference candidates, without training or actuator I/O."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'work/plot_dependencies')]
from landing.constrained_search import search_references,select_reference

def main():
    a=argparse.ArgumentParser();a.add_argument('--output',type=Path,required=True)
    args=a.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    cfg=json.loads((ROOT/'config/experiments/rapid_contact_v1.json').read_text())
    report=search_references(ROOT,cfg,[.25,.30,.35,.45],[4.5,4.75,5.,5.25,5.5])
    report['selection_at_target_speed_caps']={str(cap):select_reference(report,cap) for cap in [.25,.30,.35,.45]}
    (args.output/'search.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({'candidate_count':len(report['candidates']),'reference_feasible_count':sum(c['reference_sampled_feasible'] for c in report['candidates']),
        'selections':{k:None if c is None else {key:c[key] for key in ['duration_s','target_closing_speed_m_s','braking']} for k,c in report['selection_at_target_speed_caps'].items()}},indent=2))

if __name__=='__main__':main()
