// Uses the user-provided AgileX/Weston Robot SDK without running its motion demos.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <cerrno>
#include <ctime>
#include <unistd.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tracer_msgs/msg/tracer_status.hpp>
#include <ugv_sdk/mobile_robot/tracer_robot.hpp>
#include <ugv_sdk/details/protocol_v2/protocol_v2_parser.hpp>

using Clock=std::chrono::steady_clock;
class SafeBase:public rclcpp::Node {
 public:
  SafeBase():Node("tracer_safe_base") {
    const auto enabled=declare_parameter("enable_drive",false);
    const auto port=declare_parameter<std::string>("can_interface","can0");
    zero_only_=declare_parameter("zero_only",false);
    // Observed chassis status is 5 Hz, whereas wheel feedback is 50 Hz.
    // Keep the fast motion watchdog; allow three periods for status only.
    system_timeout_=declare_parameter("system_feedback_timeout_s",.6);
    tx_period_=declare_parameter("can_tx_period_s",.05);
    max_linear_=declare_parameter("max_linear_mps",.05);
    if(!std::isfinite(max_linear_)||max_linear_<=0.||max_linear_>.10)
      throw std::runtime_error("Invalid linear speed limit: maximum supported is 0.10 m/s");
    if(!std::isfinite(system_timeout_)||!std::isfinite(tx_period_)||system_timeout_<.3||system_timeout_>.6||tx_period_<.02||tx_period_>.05)
      throw std::runtime_error("Invalid CAN timing bounds");
    if(!enabled) throw std::runtime_error("Hardware driver disabled; use dry-run navigation");
    monitor_=socket(PF_CAN,SOCK_RAW|SOCK_NONBLOCK,CAN_RAW);
    if(monitor_<0) throw std::runtime_error("Cannot create read-only CAN feedback monitor");
    struct ifreq request{};
    if(port.size()>=IFNAMSIZ) throw std::runtime_error("Invalid CAN interface name");
    std::strncpy(request.ifr_name,port.c_str(),IFNAMSIZ-1);
    if(ioctl(monitor_,SIOCGIFINDEX,&request)<0) throw std::runtime_error("CAN interface missing");
    struct sockaddr_can address{};address.can_family=AF_CAN;address.can_ifindex=request.ifr_ifindex;
    if(bind(monitor_,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0) throw std::runtime_error("Cannot bind CAN feedback monitor");
    struct can_filter filters[2]{};
    filters[0].can_id=0x211;filters[1].can_id=0x221;
    filters[0].can_mask=filters[1].can_mask=CAN_SFF_MASK|CAN_EFF_FLAG|CAN_RTR_FLAG;
    if(setsockopt(monitor_,SOL_CAN_RAW,CAN_RAW_FILTER,filters,sizeof(filters))<0) throw std::runtime_error("CAN filter setup failed");
    int timestamps=1;
    if(setsockopt(monitor_,SOL_SOCKET,SO_TIMESTAMPNS,&timestamps,sizeof(timestamps))<0)
      throw std::runtime_error("CAN kernel receive timestamps unavailable");
    // Reuse the supplied SDK codec, with synchronous atomic SocketCAN frames.
    // Its async transport joins its own I/O thread on a read error and passes
    // stack-frame storage to async writes; neither is appropriate here.
    AgxMessage mode{};mode.type=AgxMsgControlModeConfig;
    mode.body.control_mode_config_msg.mode=CONTROL_MODE_CAN;
    if(!send(mode)) throw std::runtime_error("Cannot request CAN control mode");
    command_sub_=create_subscription<geometry_msgs::msg::Twist>("/tracer/cmd_vel",10,[this](geometry_msgs::msg::Twist::SharedPtr msg){
      bool valid=std::isfinite(msg->linear.x)&&std::isfinite(msg->angular.z)&&
        msg->linear.y==0&&msg->linear.z==0&&msg->angular.x==0&&msg->angular.y==0;
      linear_=valid&&!zero_only_?std::max(0.,std::min(max_linear_,msg->linear.x)):0.;
      angular_=valid&&!zero_only_?std::max(-.15,std::min(.15,msg->angular.z)):0.;
      command_time_=Clock::now();
    });
    ready_pub_=create_publisher<std_msgs::msg::Bool>("/tracer/ready",10);
    diagnostics_pub_=create_publisher<std_msgs::msg::String>("/tracer/driver_diagnostics",10);
    status_pub_=create_publisher<tracer_msgs::msg::TracerStatus>("/tracer_status",10);
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/tracer/wheel_odom",10);
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    last_tick_=Clock::now();
  }
  ~SafeBase() override {
    if(monitor_>=0) {motion(0.,0.);motion(0.,0.);}
    if(monitor_>=0) close(monitor_);
  }
 private:
  bool send(const AgxMessage&msg){
    can_frame frame{};
    if(!parser_.EncodeMessage(&msg,&frame)){last_send_errno_=-1;return false;}
    const auto written=write(monitor_,&frame,sizeof(frame));
    last_send_errno_=written==sizeof(frame)?0:errno;
    return written==sizeof(frame);
  }
  bool motion(double v,double w){
    AgxMessage msg{};msg.type=AgxMsgMotionCommand;
    msg.body.motion_command_msg.linear_velocity=v;msg.body.motion_command_msg.angular_velocity=w;
    return send(msg);
  }
  static double age(Clock::time_point now,Clock::time_point previous){return std::chrono::duration<double>(now-previous).count();}
  void tick(){
    if(!rclcpp::ok()){motion(0.,0.);return;}
    struct can_frame frame{};
    for(int i=0;i<256;++i){
      alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(timespec))]{};
      iovec buffer{&frame,sizeof(frame)};msghdr header{};
      header.msg_iov=&buffer;header.msg_iovlen=1;
      header.msg_control=ancillary;header.msg_controllen=sizeof(ancillary);
      if(recvmsg(monitor_,&header,MSG_DONTWAIT)!=sizeof(frame)) break;
      if(header.msg_flags&(MSG_TRUNC|MSG_CTRUNC))continue;
      timespec received_wall{};bool timestamp_found=false;
      for(auto control=CMSG_FIRSTHDR(&header);control;control=CMSG_NXTHDR(&header,control)){
        if(control->cmsg_level==SOL_SOCKET&&control->cmsg_type==SCM_TIMESTAMPNS&&control->cmsg_len>=CMSG_LEN(sizeof(timespec))){
          std::memcpy(&received_wall,CMSG_DATA(control),sizeof(received_wall));timestamp_found=true;break;
        }
      }
      if(!timestamp_found)continue;
      timespec wall_now{};if(clock_gettime(CLOCK_REALTIME,&wall_now)!=0)continue;
      const double queued_age=double(wall_now.tv_sec-received_wall.tv_sec)+double(wall_now.tv_nsec-received_wall.tv_nsec)*1e-9;
      if(!std::isfinite(queued_age)||queued_age<-.005)continue;
      const auto received_time=Clock::now()-std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(std::max(0.,queued_age)));
      AgxMessage msg{};
      if(frame.can_dlc!=8||!parser_.DecodeMessage(&frame,&msg))continue;
      // Processing an old queued frame must never refresh its watchdog.
      if(frame.can_id==0x211&&msg.type==AgxMsgSystemState&&received_time>system_time_){system_time_=received_time;state_.system_state=msg.body.system_state_msg;}
      if(frame.can_id==0x221&&msg.type==AgxMsgMotionState&&received_time>motion_time_){motion_time_=received_time;state_.motion_state=msg.body.motion_state_msg;}
    }
    const auto now=Clock::now();const auto &state=state_;
    const double system_age=age(now,system_time_),motion_age=age(now,motion_time_);
    std::string reason="ok";
    if(system_age>=system_timeout_)reason="system_state_timeout";
    else if(motion_age>=.3)reason="wheel_feedback_timeout";
    else if(state.system_state.vehicle_state!=VEHICLE_STATE_NORMAL)reason="vehicle_not_normal";
    else if(state.system_state.control_mode!=CONTROL_MODE_CAN)reason="control_not_can";
    else if(state.system_state.error_code!=0)reason="chassis_error";
    else if(send_failed_)reason="can_write_error";
    bool ready=reason=="ok";
    bool command_fresh=age(now,command_time_)<.25;
    // Read feedback at 50 Hz, but do not burst writes after executor delays.
    // A transition from motion to zero bypasses the keepalive interval.
    const double requested_v=ready&&command_fresh?linear_:0.;
    const double requested_w=ready&&command_fresh?angular_:0.;
    const bool urgent_zero=(requested_v==0.&&requested_w==0.)&&(last_sent_v_!=0.||last_sent_w_!=0.);
    if(age(now,last_send_time_)>=tx_period_||urgent_zero||send_failed_){
      last_send_time_=now;
      if(!motion(requested_v,requested_w)){ready=false;reason="can_write_error";send_failed_=true;}
      else{last_sent_v_=requested_v;last_sent_w_=requested_w;send_failed_=false;}
    }
    std_msgs::msg::Bool flag;flag.data=ready;ready_pub_->publish(flag);
    if(reason!=last_reason_||++diagnostic_ticks_%50==0){
      std::ostringstream out;
      out<<"{\"ready\":"<<(ready?"true":"false")<<",\"reason\":\""<<reason
         <<"\",\"system_age_s\":"<<system_age<<",\"motion_age_s\":"<<motion_age
         <<",\"send_errno\":"<<last_send_errno_<<",\"control_mode\":"<<int(state.system_state.control_mode)
         <<",\"vehicle_state\":"<<int(state.system_state.vehicle_state)<<",\"error_code\":"<<state.system_state.error_code<<"}";
      std_msgs::msg::String diagnostic;diagnostic.data=out.str();diagnostics_pub_->publish(diagnostic);
      // Keep filesystem/terminal logging off the command/watchdog thread.
      // Operators receive the full reason via driver_diagnostics and mission.
      last_reason_=reason;
    }
    if(age(now,system_time_)<system_timeout_&&age(now,motion_time_)<.3){
      tracer_msgs::msg::TracerStatus status;status.header.stamp=get_clock()->now();
      status.linear_velocity=state.motion_state.linear_velocity;status.angular_velocity=state.motion_state.angular_velocity;
      status.control_mode=state.system_state.control_mode;status.error_code=state.system_state.error_code;status.battery_voltage=state.system_state.battery_voltage;
      status_pub_->publish(status);
      const double dt=std::min(.1,age(now,last_tick_));
      x_+=status.linear_velocity*std::cos(yaw_)*dt;y_+=status.linear_velocity*std::sin(yaw_)*dt;yaw_+=status.angular_velocity*dt;
      nav_msgs::msg::Odometry odom;odom.header.stamp=status.header.stamp;odom.header.frame_id="wheel_odom";odom.child_frame_id="base_link";
      odom.pose.pose.position.x=x_;odom.pose.pose.position.y=y_;odom.pose.pose.orientation.z=std::sin(yaw_/2);odom.pose.pose.orientation.w=std::cos(yaw_/2);
      odom.twist.twist.linear.x=status.linear_velocity;odom.twist.twist.angular.z=status.angular_velocity;odom_pub_->publish(odom);
      // No odom TF: FAST-LIO bridge is the sole publisher of odom->base_link.
    }
    last_tick_=now;
  }
  int monitor_=-1;
  westonrobot::ProtocolV2Parser parser_;
  westonrobot::TracerCoreState state_{};
  bool zero_only_=false;
  bool send_failed_=false;
  double system_timeout_=.6,tx_period_=.05,max_linear_=.05,last_sent_v_=0.,last_sent_w_=0.;
  Clock::time_point last_send_time_{};
  int last_send_errno_=0,diagnostic_ticks_=0;
  std::string last_reason_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<tracer_msgs::msg::TracerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Clock::time_point command_time_{},system_time_{},motion_time_{},last_tick_{};
  double linear_=0,angular_=0,x_=0,y_=0,yaw_=0;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try{auto node=std::make_shared<SafeBase>();rclcpp::spin(node);node.reset();}
  catch(const std::exception&e){RCLCPP_ERROR(rclcpp::get_logger("tracer_safe_base"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
