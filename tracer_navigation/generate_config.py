#!/usr/bin/env python3
"""Generate Nav2 configuration from the installed Humble example and static map."""
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml
from common import ROOT,settings,lcm_types,stop_polygon


def main():
    c=settings();output=ROOT/'generated';output.mkdir(exist_ok=True)
    _,_,Grid=lcm_types(c)
    source=Path(c['navigation']['static_map_lcm']);raw=source.read_bytes()
    msg=Grid.lcm_decode(raw)
    cells=np.asarray(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width)
    image=np.where(cells<0,205,np.where(cells==0,254,0)).astype(np.uint8)
    Image.fromarray(np.flipud(image)).save(output/'static_map.pgm')
    origin=msg.info.origin
    if abs(origin.orientation.x)+abs(origin.orientation.y)+abs(origin.orientation.z)>1e-6:
        raise ValueError('Rotated static grid not supported')
    map_yaml={'image':'static_map.pgm','mode':'trinary','resolution':msg.info.resolution,
              'origin':[origin.position.x,origin.position.y,0.0],'negate':0,'occupied_thresh':.65,'free_thresh':.196}
    (output/'static_map.yaml').write_text(yaml.safe_dump(map_yaml))
    p=yaml.safe_load(Path('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml').read_text())
    keep=['bt_navigator','controller_server','local_costmap','global_costmap','map_server','planner_server','velocity_smoother']
    p={k:v for k,v in p.items() if k in keep}
    for name in ('bt_navigator','controller_server','map_server','planner_server','velocity_smoother'):
        p[name]['ros__parameters']['use_sim_time']=False
    robot=c['robot'];length=robot['length_m']/2;width=robot['width_m']/2
    footprint=str([[length,width],[length,-width],[-length,-width],[-length,width]])
    height=robot['height_m'] or c['navigation']['provisional_obstacle_height_m']
    def layer():
        return {'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True,'combination_method':1,
                'footprint_clearing_enabled':True,'observation_sources':'mark clear',
                'mark':{'topic':'/points_navigation_sensor','data_type':'PointCloud2',
                        'marking':True,'clearing':True,'min_obstacle_height':.03,'max_obstacle_height':height,
                        'obstacle_min_range':0.,'obstacle_max_range':6.,'raytrace_min_range':0.,'raytrace_max_range':8.,
                        'observation_persistence':0.,'expected_update_rate':.25},
                'clear':{'topic':'/points_navigation_sensor','data_type':'PointCloud2',
                         'marking':False,'clearing':True,'min_obstacle_height':-.10,'max_obstacle_height':height,
                         'raytrace_min_range':0.,'raytrace_max_range':8.,'observation_persistence':0.,'expected_update_rate':.25}}
    for name,frame in (('local_costmap','odom'),('global_costmap','map')):
        cost={'use_sim_time':False,'global_frame':frame,'robot_base_frame':'base_link','footprint':footprint,
              'footprint_padding':.10,'resolution':.08,'transform_tolerance':.2,'track_unknown_space':True,
              'update_frequency':5. if name=='local_costmap' else 2.,'publish_frequency':2.,
              'plugins':['static_layer','obstacle_layer','inflation_layer'],
              'static_layer':{'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True,'subscribe_to_updates':False},
              'obstacle_layer':layer(),'inflation_layer':{'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':.8,'cost_scaling_factor':3.},
              'always_send_full_costmap':True}
        if name=='local_costmap': cost.update(rolling_window=True,width=8,height=8)
        if c.get('realtime_navigation',{}).get('enabled',False):
            # StaticLayer is used as a transport for repeatedly updated live grids,
            # not as the saved-map authority for local collision checking.
            local=name=='local_costmap'
            cost['plugins']=['evidence_layer','inflation_layer']
            cost.pop('static_layer',None);cost.pop('obstacle_layer',None)
            cost['evidence_layer']={'plugin':'nav2_costmap_2d::StaticLayer',
                'map_topic':'/navigation/live_map' if local else '/map',
                'map_subscribe_transient_local':True,'subscribe_to_updates':False,
                'use_maximum':False,'trinary_costmap':False,'lethal_cost_threshold':100,
                'unknown_cost_value':255,'transform_tolerance':.2}
            # Humble StaticLayer reads these from the costmap node, not its plugin namespace.
            cost.update(use_maximum=False,trinary_costmap=False,lethal_cost_threshold=100,unknown_cost_value=255)
            if local:cost.update(width=int(math.ceil(c['realtime_navigation']['window_m'])),height=int(math.ceil(c['realtime_navigation']['window_m'])))
        p[name]={name:{'ros__parameters':cost}}
    p['map_server']['ros__parameters']['yaml_filename']=str(output/'static_map.yaml')
    p['planner_server']['ros__parameters'].update(expected_planner_frequency=1.,
        GridBased={'plugin':'nav2_navfn_planner/NavfnPlanner','tolerance':.10,'use_astar':True,'allow_unknown':False})
    control=p['controller_server']['ros__parameters']
    control.update(controller_frequency=10.,odom_topic='/lio/odom',failure_tolerance=.3)
    control['progress_checker'].update(required_movement_radius=.15,movement_time_allowance=20.)
    control['general_goal_checker'].update(xy_goal_tolerance=.15,yaw_goal_tolerance=.20)
    control['FollowPath']={'plugin':'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel':robot['max_linear_mps'],'lookahead_dist':.45,'min_lookahead_dist':.30,'max_lookahead_dist':.70,
        'lookahead_time':1.5,'use_velocity_scaled_lookahead_dist':False,'transform_tolerance':.2,
        'rotate_to_heading_angular_vel':robot['max_angular_rps'],'rotate_to_heading_min_angle':.35,
        'max_angular_accel':robot['max_angular_accel_rps2'],'use_rotate_to_heading':True,'allow_reversing':False,
        'min_approach_linear_velocity':.01,'approach_velocity_scaling_dist':.40,
        'use_collision_detection':True,'max_allowed_time_to_collision_up_to_carrot':2.,
        'use_regulated_linear_velocity_scaling':True,'regulated_linear_scaling_min_radius':.9,
        'regulated_linear_scaling_min_speed':.01,'use_cost_regulated_linear_velocity_scaling':True,
        'cost_scaling_dist':.5,'cost_scaling_gain':1.,'inflation_cost_scaling_factor':3.,'max_robot_pose_search_dist':4.,
        'use_interpolation':True}
    nav=p['bt_navigator']['ros__parameters']
    nav.update(global_frame='map',robot_base_frame='base_link',odom_topic='/lio/odom',
               default_nav_to_pose_bt_xml=str(ROOT/'navigate.xml'),default_nav_through_poses_bt_xml=str(ROOT/'navigate_through.xml'))
    smooth=p['velocity_smoother']['ros__parameters']
    smooth.update(smoothing_frequency=30.,max_velocity=[robot['max_linear_mps'],0.,robot['max_angular_rps']],min_velocity=[0.,0.,-robot['max_angular_rps']],
                  max_accel=[.25,0.,.5],max_decel=[-.4,0.,-.7],velocity_timeout=.25,odom_topic='/lio/odom')
    cm=yaml.safe_load(Path('/opt/ros/humble/share/nav2_collision_monitor/params/collision_monitor_params.yaml').read_text())
    cm=cm['collision_monitor']['ros__parameters']
    cm.update(use_sim_time=False,base_frame_id='base_link',odom_frame_id='odom',cmd_vel_in_topic='/cmd_vel_smoothed',
              cmd_vel_out_topic='/cmd_vel_collision_checked',transform_tolerance=.1,source_timeout=.3,
              base_shift_correction=False,stop_pub_timeout=86400.,polygons=['PolygonStop','PolygonSlow'],observation_sources=['pointcloud'])
    cm['PolygonStop'].update(points=stop_polygon(c).ravel().tolist(),max_points=1)
    cm['PolygonSlow'].update(points=[1.1,1.,1.1,-1.,-1.1,-1.,-1.1,1.],max_points=1,slowdown_ratio=.3)
    cm['pointcloud'].update(topic='/points_navigation',min_height=.03,max_height=height,enabled=True)
    p['collision_monitor']={'ros__parameters':cm}
    p['behavior_server']={'ros__parameters':{'use_sim_time':False,'global_frame':'odom','robot_base_frame':'base_link',
        'transform_tolerance':.2,'cycle_frequency':10.,'behavior_plugins':['wait'],
        'costmap_topic':'/local_costmap/costmap_raw','footprint_topic':'/local_costmap/published_footprint',
        'wait':{'plugin':'nav2_behaviors/Wait'}}}
    (output/'nav2.yaml').write_text(yaml.safe_dump(p,sort_keys=False),encoding='utf-8')
    (output/'map_source.json').write_text(json.dumps({'source':str(source),'sha256':hashlib.sha256(raw).hexdigest(),
         'known_free':int((cells==0).sum()),'unknown':int((cells<0).sum()),'fixed_map':True},indent=2))
    print('Generated static map and Humble Nav2/CollisionMonitor configuration.',flush=True)


if __name__=='__main__':main()
