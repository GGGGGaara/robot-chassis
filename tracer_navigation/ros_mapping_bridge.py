"""Model adapter isolated from the latency-critical odometry/obstacle bridge."""
import json
import re
import threading
import time
import numpy as np
import lcm
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from common import settings,lcm_types,transform_matrix,cloud_xyz
from dimos_ros_bridge import Bridge


class MappingBridge(Node):
    def __init__(self):
        super().__init__('mapping_model_adapter')
        self.TF,self.Cloud,_=lcm_types(settings());self.lock=threading.Lock();self.models={};self.fix=None;self.running=True
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.pubs={key:self.create_publisher(PointCloud2,'/mapping/'+key+'_points',qos) for key in ('global','local')}
        self.status=self.create_publisher(String,'/mapping/status',10)
        self.create_timer(.2,self.tick)
        self.worker=threading.Thread(target=self.listen,daemon=True);self.worker.start()

    def listen(self):
        bus=lcm.LCM()
        def tf(channel,raw):
            for item in self.TF.lcm_decode(raw).transforms:
                stamp=item.header.stamp.sec+item.header.stamp.nsec*1e-9
                if (item.header.frame_id,item.child_frame_id)==('odom','map') and abs(time.time()-stamp)<.5:
                    with self.lock:self.fix=(transform_matrix(item.transform),time.monotonic())
        def model(key,raw):
            with self.lock:self.models[key]=raw
        bus.subscribe(re.escape('/tf#tf2_msgs.TFMessage'),tf)
        for key in ('global','local'):
            bus.subscribe(re.escape('/'+key+'_map#sensor_msgs.PointCloud2'),lambda ch,raw,k=key:model(k,raw))
        while self.running:bus.handle_timeout(100)

    def tick(self):
        with self.lock:fix=self.fix;models=self.models;self.models={}
        if not fix or time.monotonic()-fix[1]>.5:return
        matrix=np.linalg.inv(fix[0])
        for key,raw in models.items():
            try:
                cloud=self.Cloud.lcm_decode(raw);stamp=cloud.header.stamp.sec+cloud.header.stamp.nsec*1e-9
                if cloud.header.frame_id!='odom' or not 0<=time.time()-stamp<2.:continue
                xyz=cloud_xyz(cloud)@matrix[:3,:3].T+matrix[:3,3]
                self.pubs[key].publish(Bridge.ros_cloud(xyz,'map',stamp))
                self.status.publish(String(data=json.dumps({'stream':key,'frame':'map','units':'m','points':len(xyz),'source_stamp':stamp,'timestamp':time.time()})))
            except (ValueError,TypeError) as error:self.get_logger().error(str(error))


def main():
    rclpy.init();node=MappingBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.running=False;node.worker.join(timeout=1);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
