"""Own a DimOS mapper and subscribe-only model recorder, stopping both on exit."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from zoneinfo import ZoneInfo

from common import ROOT


def main():
    p=argparse.ArgumentParser(description='Fixed-map localization + continuous 3D reconstruction. No chassis driver.')
    p.add_argument('--check',action='store_true',help='Validate the mapper command without starting any sensor')
    args=p.parse_args()
    script=ROOT.parent/'realtime_mapping_usage/start_fixed_map_relocalization.sh'
    if args.check:
        subprocess.run(['bash',str(script),'--mapping','--check'],check=True);return
    stamp=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    folder=ROOT/'model_sessions'/f'{stamp}_{uuid.uuid4().hex[:8]}'
    folder.mkdir(parents=True)
    source=ROOT/'robot.yaml'
    (folder/'robot.yaml').write_bytes(source.read_bytes())
    manifest={'created_at':time.time(),'timezone':'Asia/Shanghai','no_chassis_driver':True,
              'purpose':'Fixed reference localization with live 3D model; known-map navigation and fresh obstacle avoidance',
              'session':str(folder),'status':'starting'}
    children=[];interrupted=False
    def stop(*_):
        nonlocal interrupted
        interrupted=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    try:
        mapper=subprocess.Popen(['bash',str(script),'--mapping'],start_new_session=True)
        children.append(mapper)
        recorder=subprocess.Popen([sys.executable,str(ROOT/'model_recorder.py'),'--output',str(folder)],start_new_session=True)
        children.append(recorder)
        manifest['status']='running'
        (folder/'session.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        print(f'Live model session: {folder}\nKeep this terminal open. Ctrl+C saves the last model and stops this session.',flush=True)
        while not interrupted:
            if any(child.poll() is not None for child in children):
                raise RuntimeError('Mapper or recorder exited. Check this terminal; navigation must stop on lost sensor health.')
            time.sleep(.2)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                import os
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=10)
        manifest['status']='stopped';manifest['stopped_at']=time.time()
        manifest['snapshot_available']=(folder/'latest.json').exists()
        (folder/'session.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')


if __name__=='__main__':
    try:main()
    except (RuntimeError,subprocess.CalledProcessError) as error:raise SystemExit(str(error))
