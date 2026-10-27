"""Operator-triggered relative navigation; plan, freeze target, countdown, execute."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from common import ROOT, acceptance_error, settings
from coordinate_geometry import finite_vector


def main():
    parser=argparse.ArgumentParser(description='Forward/left metres from current chassis centre. Plan only by default.')
    parser.add_argument('forward_m',type=float)
    parser.add_argument('left_m',type=float)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--timeout-s',type=float,default=180.)
    args=parser.parse_args()
    offset=finite_vector([args.forward_m,args.left_m],2,'Forward/left offset')
    config=settings();limit=float(config['navigation']['max_route_m'])
    if not .05<=math.hypot(*offset)<=limit:parser.error(f'Displacement must be between 0.05 and {limit:g} metres')
    if not math.isfinite(args.timeout_s) or not 5<=args.timeout_s<=600:parser.error('Timeout must be 5..600 seconds')
    if args.execute:
        error=acceptance_error(config)
        if error:raise RuntimeError('Site acceptance: '+error)
    folder=ROOT/'results/coordinate_targets';folder.mkdir(parents=True,exist_ok=True)
    # Kernel-released lock: an interrupted countdown cannot leave a stale lock.
    with (folder/f'.operator_{os.environ.get("ROS_DOMAIN_ID","42")}.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        target=folder/f'{time.time_ns()}_{uuid.uuid4().hex[:8]}.json'
        plan=[sys.executable,str(ROOT/'mission.py'),'--relative',*map(str,offset),'--save-target',str(target)]
        subprocess.run(plan,check=True)
        print(f'Frozen target saved: {target}',flush=True)
        if not args.execute:
            print('Plan only. No arming and no movement. --execute resolves a NEW target from the then-current chassis pose.',flush=True)
            return
        print('20 seconds until rechecking and attempting this target. Step out of the swept area, leave cable slack, keep physical estop ready. Ctrl+C cancels.',flush=True)
        for remaining in (20,15,10,5):
            print(f'Countdown: {remaining} s',flush=True);time.sleep(5)
        # Anchor validation refuses if the vehicle moved during the countdown.
        # All original mission, CAN, collision and acceptance checks still apply.
        subprocess.run([sys.executable,str(ROOT/'mission.py'),'--target-file',str(target),
                        '--execute','--timeout-s',str(args.timeout_s)],check=True)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:raise SystemExit('Cancelled. No automatic retry.')
    except (ValueError,RuntimeError,OSError,subprocess.CalledProcessError) as error:
        raise SystemExit('Coordinate task stopped: '+str(error))
