"""Operator-started, bounded commissioning pulse. Never an autonomous navigation goal."""
import argparse
import json
import math
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from tracer_msgs.msg import TracerStatus
from common import ROOT,settings,config_fingerprint,driver_fingerprint,software_fingerprint,pulse_profile,cloud_xyz,near_stop_count


class Pulse(Node):
    def __init__(self):
        super().__init__('ground_test_pulse')
        self.flags={};self.checked=None;self.safety=None;self.feedback=None
        self.input=self.create_publisher(Twist,'/cmd_vel_nav',10)
        self.output=self.create_publisher(Twist,'/tracer/cmd_vel',10)
        for topic,key in (('/navigation/localization_valid','pose'),('/navigation/perception_valid','cloud'),('/tracer/ready','feedback')):
            self.create_subscription(Bool,topic,lambda m,k=key:self.flags.update({k:(time.monotonic(),m.data)}),10)
        self.create_subscription(Twist,'/cmd_vel_collision_checked',lambda m:setattr(self,'checked',(time.monotonic(),m)),10)
        self.create_subscription(String,'/navigation/safety_status',lambda m:setattr(self,'safety',(time.monotonic(),json.loads(m.data))),10)
        self.create_subscription(TracerStatus,'/tracer_status',lambda m:setattr(self,'feedback',(time.monotonic(),m)),10)
        self.active_goals=False
        self.cloud_time=0.;self.near_points=0;self.config=settings()
        self.create_subscription(PointCloud2,'/points_navigation',self.cloud,qos_profile_sensor_data)
        from action_msgs.msg import GoalStatusArray
        self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',
            lambda m:setattr(self,'active_goals',any(s.status in (1,2,3) for s in m.status_list)),10)

    def step(self,v=0.,w=0.):
        msg=Twist();msg.linear.x=v;msg.angular.z=w;self.input.publish(msg)
        rclpy.spin_once(self,timeout_sec=.02)

    def cloud(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if msg.header.frame_id!='base_link' or abs(time.time()-stamp)>.25:return
        self.near_points=near_stop_count(cloud_xyz(msg),self.config);self.cloud_time=time.monotonic()

    def zero(self):
        self.input.publish(Twist());self.output.publish(Twist())

    def health(self):
        now=time.monotonic()
        if self.active_goals:return 'navigation_goal_active'
        if now-self.cloud_time>.25:return 'actual_pointcloud_stale'
        if self.near_points>1:return 'collision_zone_blocked'
        if not self.safety or now-self.safety[0]>.35:return 'navigation_gate_missing'
        gate=self.safety[1]
        if gate.get('mode')!='dry-run' or gate.get('armed'):return 'navigation_not_disarmed_preview'
        if gate.get('reason')=='estop':return 'software_estop'
        for key in ('pose','cloud','feedback'):
            stamp,valid=self.flags.get(key,(0.,False))
            if not valid or now-stamp>.3:return key+'_not_ready'
        if not self.checked or now-self.checked[0]>.15:return 'collision_output_missing'
        if not self.feedback or now-self.feedback[0]>.3:return 'wheel_feedback_stale'
        return None


def main():
    parser=argparse.ArgumentParser(description='One forward pulse: max 0.03 m/s for 1 second; operator must hold physical estop.')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--direction',choices=['forward','left','right'],default='forward')
    args=parser.parse_args()
    if not args.execute:
        print('No hardware action. To start the prepared physical test, use ground_test.sh --execute from the operator terminal.');return
    target=pulse_profile(args.direction)
    rclpy.init();node=Pulse();driver=None;report={'pulse_limit_s':1.,'linear_limit_mps':.03,'angular_limit_rps':.10,
        'direction':args.direction,'autonomous_navigation':False,'test_platform':'physical_can0','timestamp':time.time(),
        'config_sha256':config_fingerprint(settings()),'driver_sha256':driver_fingerprint(),'software_sha256':software_fingerprint()}
    log=(ROOT/'ground_test_driver.log').open('w')
    try:
        # This node owns one hardware publisher; no other controller may own one.
        for _ in range(30):rclpy.spin_once(node,timeout_sec=.02)
        if node.count_publishers('/tracer/cmd_vel')!=1:raise RuntimeError('Another hardware command publisher exists')
        if node.count_publishers('/tracer/ready')!=0:raise RuntimeError('A hardware driver is already running')
        if not node.safety or node.safety[1].get('mode')!='dry-run' or node.safety[1].get('armed'):
            raise RuntimeError('Start disarmed navigation preview first')
        if node.active_goals:raise RuntimeError('Cancel the navigation goal first')
        from ament_index_python.packages import get_package_prefix
        executable=get_package_prefix('tracer_navigation_driver')+'/lib/tracer_navigation_driver/tracer_safe_base'
        driver=subprocess.Popen([executable,'--ros-args','-p','enable_drive:=true',
            '-p',f'can_tx_period_s:={node.config["robot"]["can_tx_period_s"]}',
            '-p',f'system_feedback_timeout_s:={node.config["robot"]["system_feedback_timeout_s"]}'],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            node.output.publish(Twist());node.step(*target)
            if driver.poll() is not None:raise RuntimeError('Driver exited; inspect ground_test_driver.log')
            if node.health() is None and (abs(node.checked[1].linear.x)+abs(node.checked[1].angular.z))>.001:break
        if node.health():raise RuntimeError(node.health())
        if abs(node.checked[1].linear.x)+abs(node.checked[1].angular.z)<=.001:raise RuntimeError('collision_zone_blocked: clear people and objects around the robot')
        if abs(node.feedback[1].linear_velocity)>.01 or abs(node.feedback[1].angular_velocity)>.02:
            raise RuntimeError('Robot not stationary before test')
        print('底盘已接入，零速等待 3 秒。人员避开车前和车侧，手持物理急停；Ctrl+C 取消。',flush=True)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            node.output.publish(Twist());node.step(*target)
            if node.health():raise RuntimeError(node.health())
            if abs(node.checked[1].linear.x)+abs(node.checked[1].angular.z)<=.001:raise RuntimeError('collision_zone_blocked')
        print(f'开始一次 {args.direction} 测试：前进上限 0.03 m/s、转动上限 0.10 rad/s，最长 1 秒。',flush=True)
        start=time.monotonic();speeds=[];turns=[];blocked=0
        while time.monotonic()-start<1.:
            node.step(*target)
            if driver.poll() is not None:raise RuntimeError('Driver exited during test')
            reason=node.health()
            if reason:raise RuntimeError(reason)
            checked=node.checked[1]
            values=(checked.linear.x,checked.angular.z)
            if not all(math.isfinite(v) for v in values):raise RuntimeError('Invalid collision-chain command')
            if args.direction=='forward':
                if abs(checked.angular.z)>.001:raise RuntimeError('Unexpected turn command')
                v=max(0.,min(.03,checked.linear.x));w=0.
            else:
                if abs(checked.linear.x)>.001 or checked.angular.z*target[1]<-1e-6:raise RuntimeError('Unexpected translation/turn direction')
                v=0.;w=max(-.10,min(.10,checked.angular.z))
            msg=Twist();msg.linear.x=v;msg.angular.z=w;node.output.publish(msg)
            if abs(v)+abs(w)<=1e-6:blocked+=1
            speeds.append(float(node.feedback[1].linear_velocity))
            turns.append(float(node.feedback[1].angular_velocity))
        movement=(max(speeds,default=0.)>.005 and min(speeds,default=0.)>-.005) if args.direction=='forward' else max((w*(1 if target[1]>0 else -1) for w in turns),default=0.)>.015
        node.zero();report.update(pulse_completed=True,wheel_speed_samples=speeds,wheel_angular_samples=turns,blocked_cycles=blocked,movement_observed=movement)
        print('脉冲结束，持续发送零速并检查反馈。',flush=True)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:node.zero();node.step()
        now=time.monotonic()
        stopped=bool(node.feedback and now-node.feedback[0]<.3 and abs(node.feedback[1].linear_velocity)<.01 and abs(node.feedback[1].angular_velocity)<.02)
        report['wheel_feedback_stopped']=stopped
        if not stopped:raise RuntimeError('Stop feedback not confirmed: use physical estop immediately')
        report['passed']=bool(movement)
        if not movement:raise RuntimeError('No clear wheel motion detected; do not increase speed automatically')
        print('轮速反馈已回到零附近。请现场核对方向和实际停车情况。',flush=True)
    except (Exception,KeyboardInterrupt) as error:
        report['error']=repr(error);node.zero()
        print('测试停止：'+str(error)+'；如车辆未停，立即按物理急停。',flush=True)
    finally:
        if rclpy.ok():
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:node.zero();rclpy.spin_once(node,timeout_sec=.02)
        if driver is not None and driver.poll() is None:
            subprocess.run(['kill','-INT',str(driver.pid)],check=False)
            try:driver.wait(timeout=4)
            except subprocess.TimeoutExpired:
                # Keep a possibly live SDK process sending zero rather than
                # kill it and assume an untested firmware timeout will brake.
                report['driver_shutdown_pending']=True
                print('驱动未退出；立即确认车辆静止并按物理急停，再处理驱动。',flush=True)
        (ROOT/'ground_test_result.json').write_text(json.dumps(report,indent=2))
        (ROOT/f'ground_test_{args.direction}.json').write_text(json.dumps(report,indent=2))
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
