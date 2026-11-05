import argparse
import json
from .io import predict_file


def main():
    parser=argparse.ArgumentParser(description='Pose-only UKF/IMM vehicle prediction; no simulation or control execution')
    parser.add_argument('--input',required=True)
    parser.add_argument('--output',required=True,help='New file; existing outputs are not overwritten')
    parser.add_argument('--every',type=float,default=.2)
    parser.add_argument('--offset',type=float,nargs=2,default=(0.,0.),metavar=('DX','DY'))
    parser.add_argument('--particles',type=int,default=12)
    parser.add_argument('--horizon',type=float,default=2.)
    parser.add_argument('--step',type=float,default=.1)
    parser.add_argument('--seed',type=int,default=0)
    args=parser.parse_args()
    try:
        result=predict_file(args.input,args.output,every=args.every,offset=args.offset,
            particles_per_mode=args.particles,horizon=args.horizon,step_s=args.step,seed=args.seed)
    except (OSError,ValueError,TypeError) as exc:
        parser.exit(2,f'Error: {exc}\n')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
