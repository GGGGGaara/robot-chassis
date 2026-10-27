"""ROS2 owner for the existing DimOS FAST-LIO2/ray-tracing backend."""
import json
import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common import ROOT


class Backend(Node):
    def __init__(self):
        super().__init__('mapping_backend')
        self.declare_parameter('start_backend',True)
        self.child=None;self.started=time.time()
        self.publisher=self.create_publisher(String,'/mapping/backend_status',10)
        if self.get_parameter('start_backend').value:
            if subprocess.run(['pgrep','-x','fastlio2_native'],capture_output=True).returncode==0:
                raise RuntimeError('An existing FAST-LIO2 instance must be stopped before unified bringup')
            self.child=subprocess.Popen(['bash',str(ROOT.parent/'realtime_mapping_usage/start_fixed_map_relocalization.sh'),'--mapping'],start_new_session=True)
        self.create_timer(.5,self.tick)

    def tick(self):
        code=self.child.poll() if self.child else None
        status='external_backend' if not self.child else ('running' if code is None else 'failed')
        self.publisher.publish(String(data=json.dumps({'status':status,'pid':self.child.pid if self.child else None,
                                                      'exit_code':code,'started_at':self.started,'timestamp':time.time()})))
        if code is not None:raise RuntimeError(f'Mapping backend exited: {code}')

    def close(self):
        if self.child and self.child.poll() is None:
            os.killpg(self.child.pid,signal.SIGINT)
            try:self.child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid,signal.SIGTERM);self.child.wait(timeout=10)


def main():
    rclpy.init();node=None
    try:node=Backend();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
