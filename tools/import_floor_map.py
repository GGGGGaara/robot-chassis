"""Offline pointcloud -> immutable floor reference assets. Never starts a sensor/robot."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.ndimage import binary_dilation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tracer_navigation'))
from common import settings,lcm_types,cloud_xyz


def build_reference(points,ground_z,resolution=.1,fill_radius=.15,robot_height=1.2):
    xyz=np.asarray(points,dtype=float)
    if xyz.ndim!=2 or xyz.shape[1]!=3:raise ValueError('Expected metric N x 3 pointcloud')
    xyz=xyz[np.isfinite(xyz).all(axis=1)].copy();xyz[:,2]-=ground_z
    if len(xyz)<100:raise ValueError('Insufficient points')
    lo=np.floor((xyz[:,:2].min(axis=0)-.5)/resolution)*resolution
    shape=np.ceil((xyz[:,:2].max(axis=0)+.5-lo)/resolution).astype(int)
    if (shape<=0).any() or shape.prod()>10_000_000:raise ValueError('Unexpected floor extent; verify metre units and floor selection')
    cells=np.floor((xyz[:,:2]-lo)/resolution).astype(int)
    floor=np.abs(xyz[:,2])<=.05
    if floor.sum()<30:raise ValueError('No adequate horizontal ground evidence; check --ground-z / gravity alignment')
    free=np.zeros((shape[1],shape[0]),dtype=bool);free[cells[floor,1],cells[floor,0]]=True
    radius=int(np.ceil(fill_radius/resolution))
    if radius:
        yy,xx=np.mgrid[-radius:radius+1,-radius:radius+1]
        free=binary_dilation(free,structure=(xx*xx+yy*yy)*resolution**2<=fill_radius**2+1e-9)
    grid=np.full(free.shape,-1,dtype=np.int8);grid[free]=0
    obstacles=(xyz[:,2]>=.08)&(xyz[:,2]<=robot_height)
    grid[cells[obstacles,1],cells[obstacles,0]]=100
    return xyz.astype('<f4'),grid,lo


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path);parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--ground-z',required=True,type=float,help='Measured horizontal floor elevation in input metric frame')
    parser.add_argument('--resolution',type=float,default=.1);parser.add_argument('--fill-radius',type=float,default=.15)
    args=parser.parse_args()
    if not np.isfinite([args.ground_z,args.resolution,args.fill_radius]).all() or not .05<=args.resolution<=.2 or not 0<=args.fill_radius<=.3:
        parser.error('Invalid metric rasterization parameters')
    if args.output_dir.exists():raise SystemExit('Output already exists; reference assets are never overwritten')
    _,Cloud,Grid=lcm_types(settings());suffix=args.input.suffix.lower()
    if suffix=='.lcm':points=cloud_xyz(Cloud.lcm_decode(args.input.read_bytes()))
    elif suffix=='.npy':points=np.load(args.input,allow_pickle=False)
    elif suffix in ('.pcd','.ply'):
        import open3d as o3d
        points=np.asarray(o3d.io.read_point_cloud(str(args.input)).points)
    else:raise SystemExit('Use metric, gravity-aligned PLY, PCD, .pc2.lcm, or N x 3 .npy')
    xyz,grid,origin=build_reference(points,args.ground_z,args.resolution,args.fill_radius,float(settings()['robot']['height_m']))
    from dimos_lcm.sensor_msgs.PointField import PointField
    cloud=Cloud();cloud.header.frame_id='map';cloud.width=len(xyz);cloud.height=1;cloud.is_dense=True;cloud.is_bigendian=False
    cloud.point_step=12;cloud.row_step=len(xyz)*12;cloud.fields=[]
    for i,k in enumerate('xyz'):
        field=PointField();field.name=k;field.offset=i*4;field.datatype=7;field.count=1;cloud.fields.append(field)
    cloud.fields_length=3;cloud.data=xyz.tobytes();cloud.data_length=len(cloud.data)
    occupancy=Grid();occupancy.header.frame_id='map';occupancy.info.resolution=args.resolution
    occupancy.info.width=grid.shape[1];occupancy.info.height=grid.shape[0]
    occupancy.info.origin.position.x=float(origin[0]);occupancy.info.origin.position.y=float(origin[1]);occupancy.info.origin.orientation.w=1.
    occupancy.data=grid.reshape(-1).tolist();occupancy.data_length=grid.size
    # Encode everything before creating the output directory, so schema failures leave no partial assets.
    files={'global_map.pc2.lcm':cloud.lcm_encode(),'global_costmap.lcm':occupancy.lcm_encode()}
    folder=args.output_dir/'raytrace';folder.mkdir(parents=True)
    for name,data in files.items():(folder/name).write_bytes(data)
    manifest={'source':str(args.input.resolve()),'source_sha256':hashlib.sha256(args.input.read_bytes()).hexdigest(),
        'frame':'map','units':'m','ground_z_subtracted_m':args.ground_z,'resolution_m':args.resolution,
        'reference_fill_radius_m':args.fill_radius,'reference_only_not_live_clearance':True,
        'points':len(xyz),'shape':list(grid.shape),'sha256':{n:hashlib.sha256(v).hexdigest() for n,v in files.items()}}
    (args.output_dir/'floor_reference.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))
    print('Set TRACER_PREMAP and TRACER_STATIC_MAP_LCM in config/local.env, then regenerate config and re-accept the site.')


if __name__=='__main__':main()
