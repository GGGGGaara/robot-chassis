"""Static-map A/B preview. LCM is subscribe-only; no robot command publisher."""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import math
from pathlib import Path
import re
import threading
import time
import sys
import subprocess
import os

import lcm
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label

from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar

HERE = Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'tracer_navigation'))
from coordinate_geometry import relative_goal, distance_metrics
DEFAULT_MAP = Path(os.environ.get('TRACER_STATIC_MAP_LCM',str(HERE.parent/'reconstruction_20261017_113920/raytrace/global_costmap.lcm'))).expanduser()


def plan_cells(grid, start, goal):
    """Use repository A*, rejecting diagonal corner cuts; fallback is 4-connected."""
    h, w = grid.grid.shape
    for x, y in (start, goal):
        if not (0 <= x < w and 0 <= y < h) or grid.grid[y, x] != 0:
            raise ValueError('A 或 B 位于障碍、未知区域或车身余量范围内，请重新选点。')
    def center(cell):
        return (grid.origin.position.x + (cell[0] + .5) * grid.resolution,
                grid.origin.position.y + (cell[1] + .5) * grid.resolution)
    path = min_cost_astar(grid, center(goal), center(start), unknown_penalty=1.0, use_cpp=False)
    if path is None:
        raise ValueError('没有连通路线。请选同一片可通行区域，或检查静态地图。')
    cells = [(int(round((p.x-grid.origin.position.x)/grid.resolution)),
              int(round((p.y-grid.origin.position.y)/grid.resolution))) for p in path.poses]
    valid = True
    for (x, y), (nx, ny) in zip(cells, cells[1:]):
        if x != nx and y != ny and (grid.grid[y, nx] != 0 or grid.grid[ny, x] != 0):
            valid = False
            break
    if not valid:
        # An alternate conservative path, rather than accepting a wall-corner cut.
        queue, parents = deque([start]), {start: None}
        while queue and goal not in parents:
            x, y = queue.popleft()
            for nxt in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
                nx, ny = nxt
                if 0 <= nx < w and 0 <= ny < h and grid.grid[ny,nx] == 0 and nxt not in parents:
                    parents[nxt] = (x,y)
                    queue.append(nxt)
        if goal not in parents:
            raise ValueError('没有不穿越障碍角点的路线。')
        cells, current = [], goal
        while current is not None:
            cells.append(current)
            current = parents[current]
        cells.reverse()
    if cells[0] != start or cells[-1] != goal or any(grid.grid[y,x] != 0 for x,y in cells):
        raise RuntimeError('路径校验失败')
    return cells


class Preview:
    def __init__(self, map_path, length, width, margin):
        self.map_path = map_path
        raw = map_path.read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        self.source = OccupancyGrid.lcm_decode(raw)
        # This is the saved mapping run's odom, identical to the premap's map
        # coordinates. This relabel is NOT applied to the new run's live odom.
        self.source.frame_id = 'map'
        q = self.source.origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6:
            raise ValueError('暂不支持具有旋转原点的栅格')
        self.length, self.width, self.margin = length, width, margin
        nav=HERE.parent/'tracer_navigation'
        geometry=json.loads(subprocess.check_output([str(nav/'.ros_venv/bin/python'),str(nav/'robot_geometry.py')],text=True))
        self.mount=np.asarray(geometry['mount'],dtype=float)
        self.radius = math.hypot(length, width)/2 + margin
        # Only confirmed zero-cost cells; unknown and intermediate costs blocked.
        known_free = self.source.grid == 0
        distance = distance_transform_edt(np.pad(known_free, 1, constant_values=False))[1:-1,1:-1]
        # Cell-diagonal allowance makes the disk/cell treatment conservative.
        self.free = distance*self.source.resolution > self.radius + self.source.resolution/math.sqrt(2)
        self.grid = OccupancyGrid(grid=np.where(self.free,0,100).astype(np.int8),
                                  resolution=self.source.resolution, origin=self.source.origin, frame_id='map')
        self.lock = threading.Lock()
        self.transforms = {}
        self.path = None
        self.stop = threading.Event()
        colors = np.empty((*self.source.grid.shape,3),dtype=np.uint8)
        colors[:] = [41,50,65]
        colors[self.source.grid >= 0] = [179,107,88]
        colors[known_free] = [98,109,121]
        colors[self.free] = [207,225,218]
        image = Image.fromarray(np.flipud(colors))
        output = BytesIO()
        image.save(output,format='PNG')
        self.png = output.getvalue()

    def cell(self, xy):
        values = np.asarray(xy,dtype=float)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError('坐标必须为两个有限数值')
        return (math.floor((values[0]-self.grid.origin.position.x)/self.grid.resolution),
                math.floor((values[1]-self.grid.origin.position.y)/self.grid.resolution))

    def world(self, cell):
        return [self.grid.origin.position.x+(cell[0]+.5)*self.grid.resolution,
                self.grid.origin.position.y+(cell[1]+.5)*self.grid.resolution]

    def state(self):
        with self.lock:
            transforms = dict(self.transforms)
            route = self.path
        now = time.monotonic()
        fix = transforms.get(('odom','map'))
        sensor = transforms.get(('odom','mid360_link'))
        pose, age = None, None
        if fix and sensor:
            age = max(now-fix[1],now-sensor[1])
            matrix = np.linalg.inv(fix[0]) @ sensor[0] @ np.linalg.inv(self.mount)
            pose = {'x':float(matrix[0,3]), 'y':float(matrix[1,3]),
                    'yaw':float(math.atan2(matrix[1,0],matrix[0,0]))}
        return {'pose':pose, 'age_s':age, 'fresh':age is not None and age < .5, 'route':route,
                'pose_reference':'chassis_center','units':'m'}

    def subscribe(self):
        def loop():
            bus = lcm.LCM()
            def receive(channel, data):
                received = time.monotonic()
                for transform in TFMessage.lcm_decode(data).transforms:
                    pair = (transform.frame_id,transform.child_frame_id)
                    if pair in (('odom','map'),('odom','mid360_link')):
                        # Ignore delayed stamped data as well as a silent stream.
                        if transform.ts and abs(time.time()-transform.ts) > 2.0:
                            continue
                        with self.lock:
                            self.transforms[pair] = (transform.to_matrix(),received)
            bus.subscribe(re.escape('/tf#tf2_msgs.TFMessage'),receive)
            while not self.stop.is_set():
                bus.handle_timeout(100)
        threading.Thread(target=loop,daemon=True).start()

    def plan(self, request):
        output=HERE/'path_preview_results';output.mkdir(exist_ok=True)
        # Invalidate the previous target on a failed replacement; never silently drive to old B.
        with self.lock:self.path=None
        (output/'latest_path.json').write_text(json.dumps({'valid':False,'planned_at':time.time()}),encoding='utf-8')
        live = request.get('live', True)
        state = self.state()
        if live:
            if not state['fresh']:
                raise ValueError('没有新鲜定位数据。保持重定位终端运行，等待匹配成功。')
            start = [state['pose']['x'],state['pose']['y']]
        else:
            start = request.get('start')
        goal = request.get('goal')
        if 'offset' in request:
            if not live or not state['fresh']:raise ValueError('相对坐标需要新鲜的实时底盘定位')
            goal=relative_goal([state['pose'][k] for k in ('x','y','yaw')],request['offset'])
        # Clear obsolete routes even when a new request is rejected.
        with self.lock:
            self.path = None
        cells = plan_cells(self.grid,self.cell(start),self.cell(goal))
        points = [self.world(cell) for cell in cells]
        distance = sum(math.dist(a,b) for a,b in zip(points,points[1:]))
        result = {'points':points,'length_m':distance,'start':start,'goal':goal,'valid':True,
                  'frame':'map','live_start':bool(live),'planned_at':time.time(),
                  'map_sha256':self.sha256,'radius_m':self.radius,
                  'note':'起点为底盘中心；静态路线预览，不发送运动控制，实车使用新鲜点云重新规划。',
                  **distance_metrics(start,goal,points)}
        with self.lock:
            self.path = result
        output = HERE/'path_preview_results'
        output.mkdir(exist_ok=True)
        (output/'latest_path.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        return result

    def metadata(self):
        return {'width':self.grid.width,'height':self.grid.height,'resolution':self.grid.resolution,
                'origin':[self.grid.origin.position.x,self.grid.origin.position.y],
                'length_m':self.length,'width_m':self.width,'radius_m':self.radius,
                'map_sha256':self.sha256,'free_cells':int(self.free.sum())}


def checks(app):
    from dimos.msgs.geometry_msgs.Pose import Pose
    test = OccupancyGrid(grid=np.zeros((15,20),dtype=np.int8),resolution=.1,origin=Pose(),frame_id='map')
    test.grid[:,10] = 100
    try:
        plan_cells(test,(2,7),(17,7))
        raise AssertionError('Crossed closed wall')
    except ValueError:
        pass
    test.grid[11,10] = 0
    cells = plan_cells(test,(2,7),(17,7))
    assert (10,11) in cells
    for (x,y),(nx,ny) in zip(cells,cells[1:]):
        assert x == nx or y == ny or (test.grid[y,nx] == 0 and test.grid[ny,x] == 0)
    labels, count = label(app.free)
    sizes = np.bincount(labels.ravel()); sizes[0] = 0
    component = np.argwhere(labels == sizes.argmax())
    if len(component) < 2:
        raise RuntimeError('静态地图膨胀后没有足够可通行区域')
    a = tuple(map(int,component[len(component)//3][::-1]))
    b = tuple(map(int,component[2*len(component)//3][::-1]))
    real = plan_cells(app.grid,a,b)
    # Validate frame composition on a known nonzero map/odom transform.
    matrix = np.eye(4); matrix[0,3] = -5
    sensor = np.eye(4); sensor[0,3] = 2; sensor=sensor@app.mount
    with app.lock:
        app.transforms = {('odom','map'):(matrix,time.monotonic()),
                          ('odom','mid360_link'):(sensor,time.monotonic())}
    assert abs(app.state()['pose']['x']-7) < 1e-9
    with app.lock:
        app.transforms.clear()
    assert not app.state()['fresh']
    report = {'passed':True,'map':app.metadata(),'components':count,'largest_component':int(sizes.max()),
              'sample_start':app.world(a),'sample_goal':app.world(b),'sample_path_nodes':len(real),
              'checks':['closed-wall rejection','doorway route','no diagonal corner cuts',
                        'static-map connected path','inverse TF composition','missing-pose rejection']}
    (HERE/'path_preview_check.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)


def serve(app,port):
    class Handler(BaseHTTPRequestHandler):
        def send(self,data,mime='application/json; charset=utf-8',status=200):
            if not isinstance(data,bytes):
                data = json.dumps(data,ensure_ascii=False,allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type',mime)
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store')
            self.end_headers(); self.wfile.write(data)
        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/': self.send((HERE/'fixed_map_path_preview.html').read_bytes(),'text/html; charset=utf-8')
            elif path == '/map.png': self.send(app.png,'image/png')
            elif path == '/meta': self.send(app.metadata())
            elif path == '/state': self.send(app.state())
            else: self.send({'error':'Not found'},status=404)
        def do_POST(self):
            # Local browser interface only. No cross-origin access or arbitrary files.
            if self.headers.get('Origin') not in (None,f'http://localhost:{port}',f'http://127.0.0.1:{port}'):
                self.send({'error':'Origin rejected'},status=403); return
            if self.path != '/plan': self.send({'error':'Not found'},status=404); return
            try:
                size = int(self.headers.get('Content-Length','0'))
                if not 0 < size < 4096: raise ValueError('请求大小不正确')
                self.send(app.plan(json.loads(self.rfile.read(size))))
            except (ValueError,TypeError,KeyError,IndexError) as error:
                self.send({'error':str(error)},status=400)
        def log_message(self,*args): pass
    app.subscribe()
    server = ThreadingHTTPServer(('127.0.0.1',port),Handler)
    print(f'固定地图路径预览: http://localhost:{port}',flush=True)
    print('只读订阅 TF，不发布路径或速度到机器人。Ctrl+C 关闭本页面服务。',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: app.stop.set(); server.server_close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map',type=Path,default=DEFAULT_MAP)
    parser.add_argument('--length',type=float,default=.702)
    parser.add_argument('--width',type=float,default=.610)
    parser.add_argument('--margin',type=float,default=.10)
    parser.add_argument('--port',type=int,default=9880)
    parser.add_argument('--check',action='store_true')
    args = parser.parse_args()
    if not 0 < args.length < 5 or not 0 < args.width < 5 or not 0 <= args.margin < 2:
        parser.error('车身尺寸或余量不正确')
    preview = Preview(args.map,args.length,args.width,args.margin)
    if args.check: checks(preview)
    else: serve(preview,args.port)
