"""ROS2-only model consumer: PointCloud2 input, atomic PLY snapshots and status."""
from datetime import datetime
import json
import os
from pathlib import Path
import time
import uuid
from zoneinfo import ZoneInfo
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from common import ROOT,cloud_xyz
from model_recorder import write_ply


class ModelRecorder(Node):
    def __init__(self):
        super().__init__('model_recorder')
        self.declare_parameter('output_dir','');self.declare_parameter('save_interval_s',10.)
        self.interval=float(self.get_parameter('save_interval_s').value)
        if not 2<=self.interval<=300:raise ValueError('Save interval must be 2..300 seconds')
        output=self.get_parameter('output_dir').value
        self.output=Path(output) if output else ROOT/'model_sessions'/('ros2_'+datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
        self.output.mkdir(parents=True,exist_ok=True)
        self.pending=None;self.last_save=0.;self.saved=[]
        self.status=self.create_publisher(String,'/mapping/recorder_status',10)
        self.create_subscription(PointCloud2,'/mapping/global_points',self.receive,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.,self.tick)
        self.get_logger().info('ROS2 model output: '+str(self.output))

    def receive(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if msg.header.frame_id=='map' and 0<=time.time()-stamp<2. and msg.width*msg.height>0:
            self.pending=(msg,stamp)

    def tick(self):
        if time.monotonic()-self.last_save>=self.interval:self.save()

    def save(self):
        if self.pending is None:return
        msg,stamp=self.pending;self.pending=None
        xyz=cloud_xyz(msg);name='model_'+str(int(stamp*1e9));temp=self.output/(name+'.tmp');final=self.output/name
        temp.mkdir();write_ply(temp/'model_map.ply',xyz)
        from rclpy.serialization import serialize_message
        (temp/'global_points.cdr').write_bytes(serialize_message(msg))
        metadata={'frame':'map','units':'m','points':len(xyz),'source_stamp':stamp,'saved_at':time.time(),
                  'snapshot':name,'ros_type':'sensor_msgs/msg/PointCloud2','transport':'ROS2','no_motion_commands':True}
        (temp/'metadata.json').write_text(json.dumps(metadata,indent=2));os.replace(temp,final)
        pointer=self.output/'latest.json.tmp';pointer.write_text(json.dumps(metadata,indent=2));os.replace(pointer,self.output/'latest.json')
        self.saved.append(final)
        if len(self.saved)>2:
            old=self.saved.pop(0)
            if old.resolve().parent!=self.output.resolve():raise RuntimeError('Snapshot path mismatch')
            for item in old.iterdir():item.unlink()
            old.rmdir()
        self.last_save=time.monotonic()
        if rclpy.ok():self.status.publish(String(data=json.dumps({'status':'saved','directory':str(final),**metadata})))


def main():
    rclpy.init();node=ModelRecorder()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.save();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
