"""Read-only geometric/visual check of a captured relocalization snapshot.

Consumes diagnose_relocalization.py outputs; never publishes ROS, LCM or CAN.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from common import ROOT, settings, lcm_types, cloud_xyz, mount_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot-dir', type=Path,
                        default=ROOT.parent/'realtime_mapping_usage/localization_diagnosis')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    source = args.snapshot_dir
    info = json.loads((source/'snapshot.json').read_text())
    if 'odom->map' not in info['transforms']:
        raise SystemExit('No accepted registration transform in this snapshot')
    output = args.output_dir or ROOT/'results'/('alignment_check_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    for name in ('snapshot.json', 'loaded_map.pc2.lcm', 'local_map.pc2.lcm'):
        shutil.copy2(source/name, output/name)
    config = settings()
    _, Cloud, _ = lcm_types(config)
    saved = cloud_xyz(Cloud.lcm_decode((output/'loaded_map.pc2.lcm').read_bytes()))
    live_msg = Cloud.lcm_decode((output/'local_map.pc2.lcm').read_bytes())
    live = cloud_xyz(live_msg)
    placement = np.linalg.inv(np.asarray(info['transforms']['odom->map']))
    aligned = live @ placement[:3,:3].T + placement[:3,3]
    pose = placement @ np.asarray(info['transforms']['odom->mid360_link']) @ np.linalg.inv(mount_matrix(config))
    distances, _ = cKDTree(saved).query(aligned)
    ground = float(np.percentile(saved[:,2], 10))
    lower, upper = ground+.25, ground+1.5
    wall_saved = saved[(saved[:,2]>lower)&(saved[:,2]<upper)]
    wall_live = aligned[(aligned[:,2]>lower)&(aligned[:,2]<upper)]
    report = {
        'read_only': True, 'no_motion_commands': True,
        'generated_at': datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        'source_stamp': live_msg.header.stamp.sec+live_msg.header.stamp.nsec*1e-9,
        'reference_points': len(saved), 'local_points': len(live),
        'registration_available': True,
        'within_0_165m_fraction': float(np.mean(distances<=.165)),
        'inlier_rmse_m': float(np.sqrt(np.mean(distances[distances<=.165]**2))),
        'reference_z_percentiles': np.percentile(saved[:,2],[0,5,10,25,50,75,95,100]).tolist(),
        'above_ground_band_z_m': [lower, upper],
        'above_ground_live_points': len(wall_live),
        'chassis_position_map_m': pose[:3,3].tolist(),
        'visual_acceptance': 'pending human confirmation; numeric scores do not prove correct place or navigation acceptance',
    }
    if len(wall_saved) and len(wall_live):
        wall_d, _ = cKDTree(wall_saved).query(wall_live)
        report['above_ground_within_0_165m_fraction'] = float(np.mean(wall_d<=.165))
        report['above_ground_median_distance_m'] = float(np.median(wall_d))
    fig, axes = plt.subplots(1,2,figsize=(14,7),dpi=150)
    for ax, title in zip(axes, ['Room structure in map coordinates', 'Structure around the current chassis']):
        ax.scatter(wall_saved[:,0],wall_saved[:,1],s=.8,c='#7d8794',alpha=.45,label='Saved map')
        ax.scatter(wall_live[:,0],wall_live[:,1],s=1.2,c='#f06428',alpha=.65,label='Registered live local map')
        ax.plot(pose[0,3],pose[1,3],'o',color='#1565c0',markersize=7,label='Current chassis')
        ax.arrow(pose[0,3],pose[1,3],pose[0,0]*.55,pose[1,0]*.55,width=.025,color='#1565c0',length_includes_head=True)
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)');ax.set_title(title)
    margin = .5
    axes[0].set_xlim(float(saved[:,0].min())-margin,float(saved[:,0].max())+margin)
    axes[0].set_ylim(float(saved[:,1].min())-margin,float(saved[:,1].max())+margin)
    axes[1].set_xlim(pose[0,3]-3.5,pose[0,3]+3.5)
    axes[1].set_ylim(pose[1,3]-3.5,pose[1,3]+3.5)
    axes[0].legend(loc='upper right',markerscale=3)
    fig.suptitle('Relocalization snapshot | grey = reference, orange = live | no drive commands',fontsize=13)
    fig.text(.5,.025, 'Above-ground height band only; inspect corners and wall directions. This is a captured frame, not a live view.',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.045,1,.94))
    fig.savefig(output/'alignment_overlay.png');plt.close(fig)
    (output/'alignment_check.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'output_dir':str(output),**report},indent=2),flush=True)


if __name__=='__main__':
    main()
