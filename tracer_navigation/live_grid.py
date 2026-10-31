"""Bounded live voxel-column evidence. A reference map can never mark this grid free."""
import math
import numpy as np


class LiveGrid:
    def __init__(self, config):
        self.robot=config['robot'];self.cfg=config['realtime_navigation']
        self.res=float(self.cfg['resolution_m']);self.ttl=float(self.cfg['evidence_ttl_s'])
        self.band=float(self.cfg['vertical_band_m']);self.height=float(self.robot['height_m'])
        self.n=int(math.ceil(float(self.cfg['window_m'])/self.res));self.bands=int(math.ceil(self.height/self.band))
        if not (.05<=self.res<=.2 and .3<=self.ttl<=5 and 20<=self.n<=300 and 1<=self.bands<=24):
            raise ValueError('Invalid live grid geometry/expiry')
        self.lo=None;self.pose=None;self.last_scan=float('-inf');self.ray_offset=0
        self.ground=np.full((self.n,self.n),-np.inf)
        self.clear=np.full((self.bands,self.n,self.n),-np.inf)
        self.hit=np.full_like(self.clear,-np.inf)

    def recenter(self, pose):
        pose=np.asarray(pose,dtype=float)
        if pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('Invalid odom pose')
        low=np.floor(pose[:2]/self.res).astype(int)-self.n//2
        if self.lo is not None:
            dx,dy=(low-self.lo).tolist()
            for name in ('ground','clear','hit'):
                a=getattr(self,name)
                if abs(dx)>=self.n or abs(dy)>=self.n:a.fill(-np.inf);continue
                a=np.roll(a,(-dy,-dx),axis=(-2,-1))
                if dx>0:a[...,-dx:]=-np.inf
                if dx<0:a[...,:-dx]=-np.inf
                if dy>0:a[...,-dy:,:]=-np.inf
                if dy<0:a[...,:-dy,:]=-np.inf
                setattr(self,name,a)
        self.lo=low;self.pose=pose

    def indices(self, xy):
        q=np.floor(np.asarray(xy)/self.res).astype(int)-self.lo
        ok=(q[:,0]>=0)&(q[:,0]<self.n)&(q[:,1]>=0)&(q[:,1]<self.n)
        return q,ok

    def update(self, base_points, sensor_origin, pose, now):
        if not math.isfinite(now) or now<=self.last_scan:raise ValueError('Scan time must increase')
        xyz=np.asarray(base_points,dtype=float);origin=np.asarray(sensor_origin,dtype=float)
        if xyz.ndim!=2 or xyz.shape[1]!=3 or origin.shape!=(3,) or not np.isfinite(origin).all():raise ValueError('Invalid cloud/origin')
        self.recenter(pose)
        own=((np.abs(xyz[:,0])<=self.robot['length_m']/2+.03)&(np.abs(xyz[:,1])<=self.robot['width_m']/2+.03)&
             (xyz[:,2]>=-.1)&(xyz[:,2]<=self.height+.03))
        outside_body=(np.abs(xyz[:,0])>self.robot['length_m']/2)|(np.abs(xyz[:,1])>self.robot['width_m']/2)
        # Self-return padding must not erase observed ground just outside the real
        # footprint. This supplies support only; full-height ray evidence is still required.
        own&=~(outside_body&(np.abs(xyz[:,2])<=float(self.cfg['ground_tolerance_m'])))
        keep=np.isfinite(xyz).all(axis=1)&~own&(np.linalg.norm(xyz-origin,axis=1)<=float(self.cfg['ray_range_m']))
        xyz=xyz[keep]
        yaw=pose[2];rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        world_xy=np.einsum('ij,kj->ik',xyz[:,:2],rotation)+np.asarray(pose[:2])
        q,inside=self.indices(world_xy)
        ground=(np.abs(xyz[:,2])<=float(self.cfg['ground_tolerance_m']))&inside
        self.ground[q[ground,1],q[ground,0]]=now
        # Ray tracing is allowed to under-sample free-space evidence, never obstacle hits.
        max_rays=int(self.cfg['max_clearing_rays'])
        stride=max(1,math.ceil(len(xyz)/max_rays))
        rays=xyz[self.ray_offset%stride::stride];self.ray_offset+=1
        if len(rays):
            delta=rays-origin
            lengths=np.linalg.norm(delta,axis=1)
            steps=max(2,int(math.ceil(float(lengths.max())/(min(self.res,self.band)*.45)))+1)
            t=np.linspace(0.,1.,steps)
            samples=origin+delta[:,None,:]*t[None,:,None]
            # A metric one-cell cutoff would discard the bottom height band for
            # short/downward rays. Trace up to (excluding) the exact return;
            # endpoint obstacle hits below are applied last and win ties.
            samples=samples[:,:-1,:].reshape(-1,3)
            xy=np.einsum('ij,kj->ik',samples[:,:2],rotation)+np.asarray(pose[:2])
            cells,ok=self.indices(xy)
            band=np.floor(samples[:,2]/self.band).astype(int)
            ok&=(band>=0)&(band<self.bands)
            index=band[ok]*self.n*self.n+cells[ok,1]*self.n+cells[ok,0]
            np.maximum.at(self.clear.reshape(-1),index,now)
        occupied=inside&(xyz[:,2]>=float(self.cfg['obstacle_min_height_m']))&(xyz[:,2]<=self.height)
        b=np.minimum(self.bands-1,np.floor(xyz[occupied,2]/self.band).astype(int))
        index=b*self.n*self.n+q[occupied,1]*self.n+q[occupied,0]
        np.maximum.at(self.hit.reshape(-1),index,now)
        self.last_scan=now

    def view(self, now):
        result=np.full((self.n,self.n),-1,dtype=np.int8)
        blocked=((now-self.hit<=self.ttl)&(self.hit>=self.clear)).any(axis=0)
        supported=(now-self.ground<=self.ttl)&((now-self.clear<=self.ttl)&(self.clear>self.hit)).all(axis=0)
        result[supported]=0;result[blocked]=100
        return result


def fuse_reference(reference, resolution, origin, live, live_origin, live_resolution, map_from_odom, prior_cost=90):
    """Reference walls are costly suggestions. Only fresh live obstacles are lethal."""
    reference=np.asarray(reference,dtype=np.int8)
    if not 1<=prior_cost<=98:raise ValueError('Reference cost must remain below lethal')
    fused=np.where(reference<0,-1,np.where(reference>=50,prior_cost,5)).astype(np.int8)
    h,w=reference.shape;live=np.asarray(live,dtype=np.int8)
    transform=np.asarray(map_from_odom,dtype=float)
    if transform.shape!=(3,3) or not np.isfinite(transform).all():raise ValueError('Invalid planar registration')
    corners=np.array([[0,0],[live.shape[1],0],[0,live.shape[0]],[live.shape[1],live.shape[0]]])*live_resolution+live_origin
    mapped=np.column_stack([corners,np.ones(4)])@transform.T
    low=np.maximum(0,np.floor((mapped[:,:2].min(axis=0)-origin)/resolution).astype(int))
    high=np.minimum([w,h],np.ceil((mapped[:,:2].max(axis=0)-origin)/resolution).astype(int)+1)
    if (high<=low).any():return fused
    yy,xx=np.mgrid[low[1]:high[1],low[0]:high[0]]
    xy=np.column_stack([origin[0]+(xx.reshape(-1)+.5)*resolution,origin[1]+(yy.reshape(-1)+.5)*resolution,np.ones(xx.size)])
    odom=np.einsum('ij,kj->ik',xy,np.linalg.inv(transform))
    cells=np.floor((odom[:,:2]-live_origin)/live_resolution).astype(int)
    ok=(cells[:,0]>=0)&(cells[:,0]<live.shape[1])&(cells[:,1]>=0)&(cells[:,1]<live.shape[0])
    ids=np.flatnonzero(ok);values=live[cells[ok,1],cells[ok,0]];known=values>=0
    fused[yy.reshape(-1)[ids[known]],xx.reshape(-1)[ids[known]]]=values[known]
    return fused


def swept_permission(grid, origin, resolution, pose, command, config):
    """All cells touched by a padded swept footprint need live clearance evidence."""
    grid=np.asarray(grid);pose=np.asarray(pose,dtype=float);v,w=command
    if not np.isfinite([*pose,v,w]).all():return 'live_invalid'
    cfg=config['realtime_navigation'];robot=config['robot'];margin=float(cfg['footprint_margin_m'])
    horizon=float(cfg['prediction_s'])
    extent=max(abs(v)*horizon+v*v/(2*.4),float(cfg['minimum_lookahead_m']) if abs(v)>1e-6 else 0.)
    horizon=max(horizon,extent/abs(v)) if abs(v)>1e-6 else horizon
    distance=max(extent,abs(w)*horizon*math.hypot(robot['length_m'],robot['width_m'])/2)
    times=np.linspace(0.,horizon,max(2,int(math.ceil(distance/(resolution*.4)))+1))
    halfx=robot['length_m']/2+margin;halfy=robot['width_m']/2+margin
    bx,by=np.meshgrid(np.linspace(-halfx,halfx,int(math.ceil(2*halfx/(resolution*.4)))+1),
                      np.linspace(-halfy,halfy,int(math.ceil(2*halfy/(resolution*.4)))+1))
    body=np.column_stack([bx.reshape(-1),by.reshape(-1)])
    samples=[]
    for t in times:
        angle=w*t
        if abs(w)>1e-6:dx=v/w*math.sin(angle);dy=v/w*(1-math.cos(angle))
        else:dx=v*t;dy=0.
        # Extend integration time rather than scaling the radius: curvature must stay physical.
        yaw=pose[2]+angle;c,s=math.cos(yaw),math.sin(yaw)
        local=body@np.array([[c,s],[-s,c]])
        heading=np.array([[math.cos(pose[2]),-math.sin(pose[2])],[math.sin(pose[2]),math.cos(pose[2])]])
        samples.append(local+pose[:2]+heading@np.array([dx,dy]))
    xy=np.concatenate(samples)
    # The actual, already occupied robot body is not an unobserved future region.
    relative=(xy-pose[:2])@np.array([[math.cos(pose[2]),-math.sin(pose[2])],[math.sin(pose[2]),math.cos(pose[2])]])
    xy=xy[(np.abs(relative[:,0])>robot['length_m']/2)|(np.abs(relative[:,1])>robot['width_m']/2)]
    cells=np.floor((xy-origin)/resolution).astype(int)
    if ((cells<0).any() or (cells[:,0]>=grid.shape[1]).any() or (cells[:,1]>=grid.shape[0]).any()):return 'live_unknown'
    values=grid[cells[:,1],cells[:,0]]
    if (values>=50).any():return 'live_obstacle'
    if (values<0).any():return 'live_unknown'
    return 'ok'
