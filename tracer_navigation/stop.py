"""Best-effort software stop; a failed service is NOT proof of physical braking."""
import time
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from action_msgs.srv import CancelGoal
from tracer_msgs.msg import TracerStatus

rclpy.init();node=rclpy.create_node('tracer_operator_stop');feedback=[]
node.create_subscription(TracerStatus,'/tracer_status',lambda m:feedback.append((time.monotonic(),m.linear_velocity,m.angular_velocity)),10)
publisher=node.create_publisher(Twist,'/tracer/cmd_vel',10)
for service,kind in (('/navigation/estop',SetBool),('/navigation/arm',SetBool),('/navigate_to_pose/_action/cancel_goal',CancelGoal)):
    client=node.create_client(kind,service)
    if not client.wait_for_service(timeout_sec=.5):print(service+' unavailable; use physical estop if needed.');continue
    req=kind.Request()
    if kind==SetBool:req.data=service.endswith('estop')
    future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=1)
    print(service+': '+('acknowledged' if future.done() and future.result() is not None else 'no acknowledgement'))
end=time.monotonic()+2
while time.monotonic()<end:publisher.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
stopped=bool(feedback and time.monotonic()-feedback[-1][0]<.3 and abs(feedback[-1][1])<.01 and abs(feedback[-1][2])<.02)
print('Wheel feedback confirms zero speed.' if stopped else 'Physical stopping NOT confirmed. Check vehicle and use physical estop.')
node.destroy_node();rclpy.shutdown()
