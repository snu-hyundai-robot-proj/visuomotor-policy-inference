#include <cmath>
#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <system_interface/srv/robot_calibration.hpp>

#include "vive_tracker_bridge/math_utils.hpp"

#define deg2rad M_PI / 180.0
#define rad2deg 180.0 / M_PI

inline tf2::Transform TransformFromRowMajor3x4_12(const double* d12)
{
  const double r00 = d12[0],  r01 = d12[1],  r02 = d12[2];
  const double r10 = d12[4],  r11 = d12[5],  r12 = d12[6];
  const double r20 = d12[8],  r21 = d12[9],  r22 = d12[10];

  const double px = d12[3];
  const double py = d12[7];
  const double pz = d12[11];

  tf2::Matrix3x3 R(r00, r01, r02,
                  r10, r11, r12,
                  r20, r21, r22);

  tf2::Vector3 p(px, py, pz);

  return tf2::Transform(R, p);
}

class ViveBridgeNode : public rclcpp::Node
{
public:
  ViveBridgeNode()
  : rclcpp::Node("tracker_bridge_node", "tracker"),
    tf_broadcaster_(*this),
    static_tf_broadcaster_(*this)
  {
    this->declare_parameter<bool>("simulation", true);
    sim = this->get_parameter("simulation").as_bool();

    pub_pose_ = create_publisher<geometry_msgs::msg::PoseArray>("tracker_states", rclcpp::QoS(rclcpp::KeepLast(1)));
    // pub_sim_ = create_publisher<geometry_msgs::msg::PoseArray>("tracker_simulate", rclcpp::QoS(rclcpp::KeepLast(1)));
    pub_sim_ = create_publisher<geometry_msgs::msg::PoseArray>("tracker_simulate", rclcpp::QoS(rclcpp::KeepLast(1)));

    sub_euler_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "tracker_euler", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&ViveBridgeNode::onEuler, this, std::placeholders::_1));

    cal_server_ = this->create_service<system_interface::srv::RobotCalibration>
    ("calibration_server", 
    std::bind(&ViveBridgeNode::on_request, this,
                std::placeholders::_1, std::placeholders::_2));

    publishStaticWorldToVive();
    if(sim) RCLCPP_INFO(get_logger(), "Sub /tracker/tracker_euler -> Pub /tracker/tracker_simulate + TF // Srv /tracker/calibration_server");
    else RCLCPP_INFO(get_logger(), "Sub /tracker/tracker_euler -> Pub /tracker/tracker_states + TF // Srv /tracker/calibration_server");

    calibration_left.setIdentity();
    calibration_right.setIdentity();

    temp_robot.setIdentity();
    cal_robot.setIdentity();
  }

  void calibrate_position(float* position)
  {
    calibration_left = recv_left;
    calibration_right = recv_right;

    tf2::Quaternion q;
    q.setRPY(position[3],position[4],position[5]);
    q.normalize();

    tf2::Vector3 t(position[0],position[1],position[2]);
    tf2::Transform temp(q,t);

    cal_robot = temp;

    is_calibrated = true;
    is_start_pub = false;
  }

private:
  static std::array<double,4> eulerToQuat(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll * 0.5 * deg2rad), sr = std::sin(roll * 0.5 * deg2rad);
    const double cp = std::cos(pitch * 0.5 * deg2rad), sp = std::sin(pitch * 0.5 * deg2rad);
    const double cy = std::cos(yaw * 0.5 * deg2rad), sy = std::sin(yaw * 0.5 * deg2rad);

    double qw = cr*cp*cy + sr*sp*sy;
    double qx = sr*cp*cy - cr*sp*sy;
    double qy = cr*sp*cy + sr*cp*sy;
    double qz = cr*cp*sy - sr*sp*cy;

    const double n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (n > 1e-12) { qx/=n; qy/=n; qz/=n; qw/=n; }
    else { qx=0; qy=0; qz=0; qw=1; }
    return {qx,qy,qz,qw};
  }

  void publishStaticWorldToVive()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "world";
    t.child_frame_id  = "tracker";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    static_tf_broadcaster_.sendTransform(t);
  }

  void publishDynamicTF(const rclcpp::Time& stamp, const std::string& child,
                        double x,double y,double z,double r,double p,double yw)
  {
    auto q = eulerToQuat(r,p,yw);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "tracker";
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    t.transform.rotation.x = q[0];
    t.transform.rotation.y = q[1];
    t.transform.rotation.z = q[2];
    t.transform.rotation.w = q[3];

    tf_broadcaster_.sendTransform(t);
  }

  void on_request(const std::shared_ptr<system_interface::srv::RobotCalibration::Request> req,
    std::shared_ptr<system_interface::srv::RobotCalibration::Response> res)
  {
    RCLCPP_INFO(get_logger(), "Get Command cmd : %ld", req->command);

    res->accepted = false;
    res->code = 999;

    if(req->side == "Left")
    {
      float current_position[6];
      if(req->command == 0) // stop
      {
        stop_publish("left");
      }
      else if(req->command == 1) // calibration
      {
        for(int i=0;i<6;i++) current_position[i] = req->robot_pose[i];
        // calibrate_position(current_position);
        calibrate_position(current_position);
      }
      else if(req->command == 2) // start publish
      {
        if(is_calibrated) is_start_pub = true;
        else RCLCPP_INFO(get_logger(),"Calibration First");
      }
      else
      {
        res->accepted = false;
        res->code = req->command;
      }

      res->accepted = true;
      res->code = req->command;
    }
    else if(req->side == "Right")
    {
      float current_position[6];
      if(req->command == 0) // stop
      {
        stop_publish("right");
      }
      else if(req->command == 1) // calibration
      {
        for(int i=0;i<6;i++) current_position[i] = req->robot_pose[i];
        // calibrate_position(current_position);
        calibrate_position(current_position);
      }
      else if(req->command == 2) // start publish
      {
        if(is_calibrated) is_start_pub = true;
        else RCLCPP_INFO(get_logger(),"Calibration First");
      }
      else
      {
        res->accepted = false;
        res->code = req->command;
      }

      res->accepted = true;
      res->code = req->command;
    }
    else
    {
      res->accepted = false;
      res->code = req->command;
    }
  }

  void onEuler(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 12) return;

    const auto stamp = now();

    bool left_data_received = false;
    bool right_data_received = false;

    for(int i=0;i<12;i++)
    {
      local_left[i] = msg->data[i];
      local_right[i] = msg->data[i+12];

      if(local_left[i] == 0.0) left_data_received = false;
      else left_data_received = true;
      if(local_right[i] == 0.0) right_data_received = false;
      else right_data_received = true;
    }

    // if(!left_data_received) // 추후 활성화 안전 설정값
    // {
    //   stop_publish()
    // }
    
    geometry_msgs::msg::PoseArray out;

    geometry_msgs::msg::Pose left_pose;
    geometry_msgs::msg::Pose right_pose;

    out.header.stamp = stamp;
    out.header.frame_id = "tracker";

    static tf2::Matrix3x3 local_align_mat(   // {-90, 0, -180}
            -1,  0,  0,
             0,  0,  1,
             0,  1,  0);
    static tf2::Vector3 local_align_vec(0,0,0);

    static tf2::Transform local_align_transform(local_align_mat, local_align_vec);

    static tf2::Matrix3x3 operator_base_mat(
            -1,  0,  0,
             0, -1,  0,
             0,  0,  1);
    static tf2::Vector3 operator_base_vec(0,0,0);

    static tf2::Transform operator_base_transform(operator_base_mat, operator_base_vec);

    {
      // ===== LEFT POSE =====
      tf2::Transform T1 = TransformFromRowMajor3x4_12(local_left);  // local_left 는 Tracker 에서 받아오는 raw Matrix 입니다.
      tf2::Transform C1;
      C1.setIdentity();

      T1 = local_align_transform.inverse() * T1;    // 트래커 + 베이스 스테이션의 로컬 좌표계를 베이스 좌표계로 변환

      recv_left = T1;     // 베이스 좌표계 상에서 트래커 위치

      // get offset
      tf2::Matrix3x3 calibrated_rotation;
      tf2::Vector3 calibrated_position;

      // cal_robot : 로봇 툴 플랜지 좌표
      // operator_base_transform : 베이스 좌표계에서 트래커 착용자 좌표계 변환 행렬 ( z축 180도 ), 현장에서는 로봇의 x축과 사용자의 x축이 마주보고 있어서 동일하게 바라보도록 변환해줬습니다.
      // calibration_left : 캘리브레이션 트리거 시 트래커 현재 좌표를 0으로 변환하기 위해 저장하는 행렬입니다. 
      // translation 과 orientation 을 분리해서 계산합니다. ( 합쳐서 계산할 시 틀어짐이 발생합니다. )
      calibrated_position = cal_robot.getOrigin()
                            + operator_base_transform.getBasis()
                            * (T1.getOrigin() - calibration_left.getOrigin());
      calibrated_rotation = operator_base_transform.getBasis() 
                            * T1.getBasis() * calibration_left.getBasis().transpose() 
                            * operator_base_transform.getBasis().transpose() * cal_robot.getBasis();

      tf2::Transform calibrated_pose(calibrated_rotation, calibrated_position);

      C1 = calibrated_pose; // 최종 변환 결과입니다.

      tf2::Matrix3x3 calrot;

      calrot = C1.getBasis();
      tf2::Vector3 trans = C1.getOrigin();

      tf2::Quaternion left_q;
      calrot.getRotation(left_q);

      // translation = 0
      left_pose.position.x = trans[0];
      left_pose.position.y = trans[1];
      left_pose.position.z = trans[2];

      // orientation 설정
      tf2::Quaternion lq = left_q;
      lq.normalize();

      left_pose.orientation.x = lq.x();
      left_pose.orientation.y = lq.y();
      left_pose.orientation.z = lq.z();
      left_pose.orientation.w = lq.w();
    }

    {
      // ===== RIGHT POSE =====
      tf2::Transform T2 = TransformFromRowMajor3x4_12(local_right);
      tf2::Transform C2;
      C2.setIdentity();

      T2 = local_align_transform.inverse() * T2;

      recv_right = T2;     // base to tracker transform

      // get offset
      tf2::Matrix3x3 calibrated_rotation;
      tf2::Vector3 calibrated_position;

      calibrated_position = cal_robot.getOrigin() 
                            + operator_base_transform.getBasis() 
                            * (T2.getOrigin() - calibration_right.getOrigin());

      calibrated_rotation = operator_base_transform.getBasis() 
                            * T2.getBasis() * calibration_right.getBasis().transpose() 
                            * operator_base_transform.getBasis().transpose() * cal_robot.getBasis();

      tf2::Transform calibrated_pose(calibrated_rotation, calibrated_position);

      C2 = calibrated_pose;

      tf2::Matrix3x3 calrot;

      calrot = C2.getBasis();
      tf2::Vector3 trans = C2.getOrigin();

      tf2::Quaternion right_q;
      calrot.getRotation(right_q);

      right_pose.position.x = trans[0];
      right_pose.position.y = trans[1];
      right_pose.position.z = trans[2];

      tf2::Quaternion rq = right_q;
      rq.normalize();

      right_pose.orientation.x = rq.x();
      right_pose.orientation.y = rq.y();
      right_pose.orientation.z = rq.z();
      right_pose.orientation.w = rq.w();
    }

    out.poses.push_back(left_pose);
    out.poses.push_back(right_pose);

    out.poses.resize(3);

    // RCLCPP_INFO(get_logger(), "pub_state : %ld, sim : %ld", is_start_pub, sim);
    if(is_start_pub 
      && !sim)
      {
        pub_pose_->publish(out);
      }

    pub_sim_->publish(out);
  }

  void stop_publish(std::string side)
  {
    if(side == "left")
    {

    }
    else if(side == "right")
    {

    }
    is_start_pub = false;
    is_calibrated = false;
  }

  bool is_calibrated = false;
  bool is_start_pub = false;
  bool sim = true;

  double local_left[12];
  double local_right[12];

  tf2::Transform recv_left;
  tf2::Transform recv_right;

  tf2::Transform calibration_left;
  tf2::Transform calibration_right;

  tf2::Transform temp_robot;
  tf2::Transform cal_robot;

  rclcpp::Service<system_interface::srv::RobotCalibration>::SharedPtr cal_server_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_sim_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_euler_;
  
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ViveBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}