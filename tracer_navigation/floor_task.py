"""Whole-floor target, frozen plan, countdown, then one guarded realtime mission."""
import argparse
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import time
from common import ROOT,settings,acceptance_error
from coordinate_geometry import finite_vector


def main():
    parser=argparse.ArgumentParser()
    target=parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--xy',type=float,nargs=2,metavar=('MAP_X','MAP_Y'))
    target.add_argument('--relative',type=float,nargs=2,metavar=('FORWARD','LEFT'))
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args();config=settings()
    if not config.get('realtime_navigation',{}).get('enabled'):raise SystemExit('Realtime profile is required')
    values=finite_vector(args.xy if args.xy is not None else args.relative,2,'Floor target')
    if args.execute:
        error=acceptance_error(config)
        if error:raise RuntimeError('Site acceptance: '+error)
    folder=ROOT/'results/floor_targets';folder.mkdir(parents=True,exist_ok=True)
    with (folder/f'.operator_{os.environ.get("ROS_DOMAIN_ID","42")}.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        frozen=folder/f'{time.time_ns()}.json'
        subprocess.run([sys.executable,str(ROOT/'mission.py'),'--floor','--xy' if args.xy is not None else '--relative',
                        *map(str,values),'--save-target',str(frozen)],check=True)
        if not args.execute:
            print('Tentative plan only. Actual reachability remains unconfirmed until live observations and arrival.');return
        print('20 seconds until rechecking one whole-floor mission. Keep cable slack and physical estop available; Ctrl+C cancels.',flush=True)
        for remaining in (20,15,10,5):print(f'Countdown: {remaining}s',flush=True);time.sleep(5)
        subprocess.run([sys.executable,str(ROOT/'mission.py'),'--floor','--target-file',str(frozen),'--execute'],check=True)


if __name__=='__main__':main()
