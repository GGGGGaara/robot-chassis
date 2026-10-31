"""Live-evidence regressions. No ROS graph, real sensor, or CAN required."""
import copy
import math
import unittest
import numpy as np
from common import settings
from live_grid import LiveGrid,fuse_reference,swept_permission
from coordinate_geometry import validate_route_length


def room_rays(cells, config):
    """Synthetic ideal lidar: first hits on floor/ceiling/four walls of a room."""
    origin=np.array([config['mount'][k] for k in ('x_m','y_m','z_m')])
    points=[];height=config['robot']['height_m'];band=config['realtime_navigation']['vertical_band_m']
    for x,y in cells:
        points.append([x,y,0.])
        for z in np.arange(math.ceil(height/band))*band+band*.85:
            vector=np.array([x,y,z])-origin
            times=[]
            for axis,low,high in ((0,-5.,5.),(1,-5.,5.),(2,0.,2.8)):
                if vector[axis]>1e-9:times.append((high-origin[axis])/vector[axis])
                elif vector[axis]<-1e-9:times.append((low-origin[axis])/vector[axis])
            t=min(v for v in times if v>0)
            points.append(origin+vector*t)
    return np.asarray(points)


class LiveEvidence(unittest.TestCase):
    def setUp(self):
        self.config=copy.deepcopy(settings());self.config['realtime_navigation']['max_clearing_rays']=100000
        self.grid=LiveGrid(self.config);self.origin=[self.config['mount'][k] for k in ('x_m','y_m','z_m')]

    def value(self,point,now):
        q,ok=self.grid.indices(np.asarray([point]));self.assertTrue(ok[0])
        return self.grid.view(now)[q[0,1],q[0,0]]

    def test_no_return_or_reference_cannot_create_clearance(self):
        self.grid.update(np.empty((0,3)),self.origin,[0,0,0],1.)
        self.assertTrue((self.grid.view(1.)==-1).all())

    def test_ground_alone_is_not_full_height_clearance(self):
        self.grid.update([[1.05,.05,0]],self.origin,[0,0,0],1.)
        self.assertEqual(self.value([1.05,.05],1.),-1)

    def test_ground_outside_actual_body_survives_self_return_padding(self):
        self.grid.update([[.36,.05,0]],self.origin,[0,0,0],1.)
        q,_=self.grid.indices(np.array([[.36,.05]]));x,y=q[0]
        self.assertEqual(self.grid.ground[y,x],1.)
        self.assertEqual(self.value([.36,.05],1.),-1)

    def test_full_vertical_rays_and_ground_allow_candidate_cell(self):
        self.grid.update(room_rays([(1.05,.05)],self.config),self.origin,[0,0,0],1.)
        self.assertEqual(self.value([1.05,.05],1.),0)

    def test_short_downward_rays_do_not_lose_low_clearance_band(self):
        self.grid.update(room_rays([(.395,.05)],self.config),self.origin,[0,0,0],1.)
        self.assertEqual(self.value([.395,.05],1.),0)

    def test_bounded_clearing_rotates_samples_instead_of_starving_height_bands(self):
        self.config['realtime_navigation']['max_clearing_rays']=2;self.grid=LiveGrid(self.config)
        cloud=room_rays([(1.05,.05)],self.config)
        for i in range(14):self.grid.update(cloud,self.origin,[0,0,0],1.+i*.05)
        self.assertEqual(self.value([1.05,.05],1.65),0)

    def test_new_obstacle_wins_same_scan_clearing(self):
        cloud=np.vstack([room_rays([(1.05,.05)],self.config),[1.05,.05,.65]])
        self.grid.update(cloud,self.origin,[0,0,0],1.)
        self.assertEqual(self.value([1.05,.05],1.),100)

    def test_removed_obstacle_needs_actual_ray_clearing(self):
        cloud=room_rays([(1.05,.05)],self.config)
        self.grid.update(np.vstack([cloud,[1.05,.05,.65]]),self.origin,[0,0,0],1.)
        self.grid.update([[1.05,.05,0]],self.origin,[0,0,0],1.1)
        self.assertEqual(self.value([1.05,.05],1.1),100)
        self.grid.update(cloud,self.origin,[0,0,0],1.2)
        self.assertEqual(self.value([1.05,.05],1.2),0)

    def test_expired_free_and_obstacle_become_unknown(self):
        self.grid.update(np.vstack([room_rays([(1.05,.05)],self.config),[2.05,.05,.6]]),self.origin,[0,0,0],1.)
        self.assertEqual(self.value([1.05,.05],4.),-1)
        self.assertEqual(self.value([2.05,.05],4.),-1)

    def test_no_floor_support_for_drop_remains_unknown(self):
        rays=room_rays([(1.05,.05)],self.config)
        rays=rays[rays[:,2]!=0]
        self.grid.update(rays,self.origin,[0,0,0],1.)
        self.assertEqual(self.value([1.05,.05],1.),-1)

    def test_overhang_inside_vehicle_height_is_obstacle(self):
        self.grid.update([[1.05,.05,1.15]],self.origin,[0,0,0],1.)
        self.assertEqual(self.value([1.05,.05],1.),100)

    def test_recenter_preserves_world_obstacle_then_discards_outside_window(self):
        self.grid.update([[1.05,.05,.6]],self.origin,[0,0,0],1.)
        self.grid.recenter([1.,0.,0]);self.assertEqual(self.value([1.05,.05],1.),100)
        self.grid.recenter([50.,0.,0]);self.assertTrue((self.grid.view(1.)==-1).all())

    def test_reference_wall_is_not_realtime_lethal(self):
        ref=np.array([[0,100,-1]],dtype=np.int8)
        out=fuse_reference(ref,.1,[0,0],np.full((1,3),-1),[0,0],.1,np.eye(3))
        self.assertEqual(out.tolist(),[[5,90,-1]])

    def test_live_replaces_both_saved_free_and_saved_wall(self):
        out=fuse_reference(np.array([[100,0,-1]]),.1,[0,0],np.array([[0,100,0]]),[0,0],.1,np.eye(3))
        self.assertEqual(out.tolist(),[[0,100,0]])

    def test_registration_places_live_observation_in_map(self):
        transform=np.array([[0.,-1.,1.],[1.,0.,0.],[0.,0.,1.]])
        out=fuse_reference(np.zeros((20,20),dtype=np.int8),.1,[0,0],np.array([[100]]),[0,0],.1,transform)
        self.assertEqual(out[0,9],100)

    def test_swept_footprint_blocks_unknown_and_obstacle(self):
        a=np.zeros((120,120),dtype=np.int8);origin=np.array([-6.,-6.])
        self.assertEqual(swept_permission(a,origin,.1,[0,0,0],(.1,0),self.config),'ok')
        a[60,66]=-1
        self.assertEqual(swept_permission(a,origin,.1,[0,0,0],(.1,0),self.config),'live_unknown')
        a[60,66]=100
        self.assertEqual(swept_permission(a,origin,.1,[0,0,0],(.1,0),self.config),'live_obstacle')

    def test_turning_checks_vehicle_width_and_outer_corners(self):
        a=np.zeros((120,120),dtype=np.int8);a[64,63]=100
        self.assertEqual(swept_permission(a,[-6,-6],.1,[0,0,0],(0,.15),self.config),'live_obstacle')

    def test_unknown_outside_window_does_not_allow_motion(self):
        self.assertEqual(swept_permission(np.zeros((3,3)),[0,0],.1,[0,0,0],(.1,0),self.config),'live_unknown')

    def test_extended_curved_lookahead_preserves_turn_radius(self):
        a=np.zeros((120,120),dtype=np.int8);a[66,65]=100
        self.assertEqual(swept_permission(a,[-6,-6],.1,[0,0,0],(.1,.15),self.config),'live_obstacle')

    def test_floor_route_bound_is_separate_from_small_range_limit(self):
        with self.assertRaises(ValueError):validate_route_length(20,30)
        validate_route_length(30,200,200)
        with self.assertRaises(RuntimeError):validate_route_length(201,200,200)


if __name__=='__main__':unittest.main(verbosity=2)
