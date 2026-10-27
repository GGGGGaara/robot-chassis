"""Recorded map/local cloud + artificial pose perturbation; NOT a live localization test."""
import json
import math
import time
from pathlib import Path
import numpy as np
import open3d as o3d
from dimos.mapping.relocalization.lidar.relocalize import LidarRelocalizer,PRESETS

ROOT=Path(__file__).resolve().parent;source=ROOT.parent/'reconstruction_20261017_113920/raytrace'
global_xyz=np.load(source/'global_map.npy');local_xyz=np.load(source/'local_map.npy')
def cloud(xyz):
    result=o3d.geometry.PointCloud();result.points=o3d.utility.Vector3dVector(xyz[:,:3]);return result
o3d.utility.random.seed(7)
config=PRESETS['mid360'].model_copy(update={'fitness_threshold':.8,'ransac_restarts':3})
start=time.monotonic();model=LidarRelocalizer(cloud(global_xyz),config)
rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
translation=np.array([3.,-2.,.1]);rng=np.random.default_rng(7)
query=local_xyz[:,:3]@rotation.T+translation+rng.normal(0.,.01,local_xyz[:,:3].shape)
result=model.align(cloud(query));expected=np.eye(4);expected[:3,:3]=rotation.T;expected[:3,3]=-rotation.T@translation
angle=math.degrees(math.acos(float(np.clip((np.trace(result.transformation[:3,:3]@expected[:3,:3].T)-1)/2,-1,1))))
error=float(np.linalg.norm(result.transformation[:3,3]-expected[:3,3]))
report={'recorded_cloud_with_artificial_perturbation':True,'live_sensor_test':False,'global_points':len(global_xyz),'local_points':len(local_xyz),
    'artificial_yaw_deg':90.,'added_noise_std_m':.01,'fitness':float(result.fitness),'rmse_m':float(result.inlier_rmse),
    'translation_error_m':error,'rotation_error_deg':angle,'elapsed_s':time.monotonic()-start,
    'estimated_transform':result.transformation.tolist(),'passed':bool(result.fitness>=.8 and error<.1 and angle<2.)}
(ROOT/'recorded_relocalization_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
