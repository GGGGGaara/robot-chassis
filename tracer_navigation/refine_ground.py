"""Small provisional floor-based mounting correction; never marks hardware verified."""
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from common import ROOT,settings,mount_matrix,ground_correction

backup=ROOT/'robot_before_ground_refinement.yaml'
c=settings()
# The observed floor is expressed using the CURRENT mounting transform.
# Applying its correction to an older backup would compound unrelated fits.
diagnosis=json.loads((ROOT/'costmap_diagnosis.json').read_text())
fit=diagnosis['floor_fit'];a,b,z=fit['a'],fit['b'],fit['c']
if fit['inliers']<200 or fit['inliers']/fit['candidates']<.80 or math.hypot(a,b)>.06 or abs(z)>.05:
    raise RuntimeError('Insufficient flat-floor evidence; do not auto-refine')
old=mount_matrix(c);correction=ground_correction(a,b,z)
new=correction@old
euler=Rotation.from_matrix(new[:3,:3]).as_euler('xyz',degrees=True)
if abs(euler[2]-c['mount']['yaw_deg'])>3 or np.linalg.norm(new[:3,3]-old[:3,3])>.05:
    raise RuntimeError('Correction changed heading/position too far; rejected')
if not backup.exists():backup.write_text((ROOT/'robot.yaml').read_text())
(ROOT/'robot_before_latest_ground_refinement.yaml').write_text((ROOT/'robot.yaml').read_text(),encoding='utf-8')
c['mount'].update(dict(zip(('x_m','y_m','z_m'),map(float,new[:3,3]))))
c['mount'].update(dict(zip(('roll_deg','pitch_deg','yaw_deg'),map(float,euler))))
c['mount']['verified']=False
c['mount']['source']='provisional flat-floor fit; physical measurements still required'
(ROOT/'robot.yaml').write_text(yaml.safe_dump(c,sort_keys=False,allow_unicode=True),encoding='utf-8')
report={'previous_mount':old.tolist(),'refined_mount':new.tolist(),'euler_degrees':euler.tolist(),
        'floor_evidence':fit,'verified_for_hardware':False}
(ROOT/'ground_refinement.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
