"""Launch an independent planar simulation session in an existing Ubuntu setup.

This starts the native supervisor/panel. The existing explicit simulation trial
authorization and its native contact/return logic are retained. No real vehicle.
"""
from pathlib import Path
import argparse
import os
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--base',type=Path,required=True,help='Existing verified Gazebo/ArduPilot environment')
    parser.add_argument('--session',type=Path,required=True,help='NEW independent session; must not exist')
    parser.add_argument('--source',type=Path,help='Original verified fixed-gimbal runtime assets')
    parser.add_argument('--frozen',type=Path,help='Original verified moving_rapid source package')
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1]/'simulation/operator_package')
    args=parser.parse_args()
    if args.session.exists():parser.error('Session exists; preserve it and choose a new path')
    env=dict(os.environ,HS_SIM_BASE=str(args.base.resolve()),HS_OPERATOR_SESSION=str(args.session.resolve()),
             HS_OPERATOR_PACKAGE=str(args.package.resolve()),
             HS_LANDING_PROFILE=str((args.package/'config/experiments/planar_landing_v1.json').resolve()))
    if args.source:env['HS_SIM_SOURCE']=str(args.source.resolve())
    if args.frozen:env['HS_FROZEN_PACKAGE']=str(args.frozen.resolve())
    required=[args.package/'operator_supervisor.py',Path(env['HS_LANDING_PROFILE'])]
    if any(not p.is_file() for p in required):parser.error('Incomplete planar source package')
    result=subprocess.run([sys.executable,str(args.package/'operator_supervisor.py')],env=env,cwd=args.package)
    return result.returncode


if __name__=='__main__':sys.exit(main())
