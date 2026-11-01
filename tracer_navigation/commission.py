"""Record site acceptance only after real, recent, successful forward/left tests."""
import argparse
import json
import time
import yaml
import hashlib
from pathlib import Path
from common import ROOT,settings,config_fingerprint,driver_fingerprint,software_fingerprint,acceptance_error,localization_map_path


def validate_evidence(c,driver_sha,reports,now):
    for direction in ('forward','left'):
        report=reports[direction]
        if report.get('test_platform')!='physical_can0' or report.get('direction')!=direction:
            raise ValueError(direction+': requires a real can0 test')
        if not all(report.get(k) for k in ('passed','pulse_completed','movement_observed','wheel_feedback_stopped')) or report.get('error'):
            raise ValueError(direction+': successful motion and stopping have not been verified')
        if report.get('config_sha256')!=config_fingerprint(c) or report.get('driver_sha256')!=driver_sha:
            raise ValueError(direction+': parameters or driver changed; repeat site test')
        if report.get('software_sha256')!=software_fingerprint():raise ValueError(direction+': navigation software changed; repeat site test')
        if not 0<=now-report.get('timestamp',0)<=12*3600:raise ValueError(direction+': site test expired')


def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['status','check','accept','revoke'])
    p.add_argument('--map-aligned',action='store_true');p.add_argument('--motion-observed',action='store_true')
    args=p.parse_args();c=settings()
    if args.command=='check':
        error=acceptance_error(c)
        if error:raise SystemExit('Site acceptance: '+error)
        print('Site acceptance valid.');return
    if args.command=='status':
        print(json.dumps({'hardware_acceptance':acceptance_error(c) or 'accepted',
            'ground_tests':{d:(ROOT/f'ground_test_{d}.json').exists() for d in ('forward','left')}},indent=2));return
    if args.command=='revoke':
        raw=yaml.safe_load((ROOT/'robot.yaml').read_text());raw['mount']['verified']=False
        (ROOT/'robot.yaml').write_text(yaml.safe_dump(raw,sort_keys=False,allow_unicode=True));print('Hardware acceptance revoked.');return
    if not args.map_aligned or not args.motion_observed:raise SystemExit('Operator must confirm map overlap and actual forward/left directions using both flags')
    now=time.time();sha=driver_fingerprint()
    try:
        reports={d:json.loads((ROOT/f'ground_test_{d}.json').read_text()) for d in ('forward','left')}
        validate_evidence(c,sha,reports,now)
    except (OSError,ValueError,KeyError) as error:raise SystemExit(str(error))
    c['mount']['verified']=True
    receipt={'timestamp':now,'config_sha256':config_fingerprint(c),'driver_sha256':sha,'software_sha256':software_fingerprint(),
        'map_sha256':hashlib.sha256(Path(c['navigation']['static_map_lcm']).read_bytes()).hexdigest(),'map_aligned':True,'motion_observed':True,
        'localization_map_sha256':hashlib.sha256(localization_map_path(c).read_bytes()).hexdigest(),
        'meaning':'Operational site acceptance of estimated extrinsics; not a metrology calibration certificate.',
        'valid_hours':18,'evidence':['ground_test_forward.json','ground_test_left.json']}
    (ROOT/'site_acceptance.json').write_text(json.dumps(receipt,indent=2))
    raw=yaml.safe_load((ROOT/'robot.yaml').read_text());raw['mount']['verified']=True
    (ROOT/'robot.yaml').write_text(yaml.safe_dump(raw,sort_keys=False,allow_unicode=True))
    print('Site acceptance recorded for 18 hours. Hardware launch still starts DISARMED.')


if __name__=='__main__':main()
