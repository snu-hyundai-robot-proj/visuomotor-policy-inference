#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <angles/angles.h>

#include <hdr_ros2_driver/utils/admittance.h>
#include <Eigen/Geometry>

struct ContinuousQuat
{
  double w{1.0};
  double x{1.0};
  double y{1.0};
  double z{1.0};
  bool initialized{false};
};

inline bool ConvertPoseArrayToPoseStamped(
    const geometry_msgs::msg::PoseArray& in,
    geometry_msgs::msg::PoseStamped& out,
    size_t idx = 0,
    bool refresh_stamp = false,
    const rclcpp::Time& now = rclcpp::Time(0, 0, RCL_ROS_TIME)  // refresh_stamp=true일 때만 사용
)
{
    if (idx >= in.poses.size())
        return false;

    out.header = in.header;

    if (refresh_stamp)
        out.header.stamp = now;

    out.pose = in.poses[idx];
  
    return true;
}

class PoseToJointStateIKNode : public rclcpp::Node
{
public:
  PoseToJointStateIKNode()
  : Node("hdr_moveit_node")
  {
    group_name_   = this->declare_parameter<std::string>("group_name", "hdr_manipulator");
    ik_link_name_ = this->declare_parameter<std::string>("ik_link_name", "tool0");
    joint_topic_  = this->declare_parameter<std::string>("joint_state_topic", "/robot/joint_target_deg");
    robot_description_  = this->declare_parameter<std::string>("robot_description", "");
    robot_srdf_ = this->declare_parameter<std::string>("robot_description_semantic", "");
    robot_side_ = this->declare_parameter<std::string>("robot_side", "left");
    // robot_kinematics_ = this->declare_parameter<std::string>("robot_description_kinematics","");

    this->get_parameter("robot_description", robot_description_);
    this->get_parameter("robot_srdf", robot_srdf_);
    // this->get_parameter("robot_kinematics", robot_kinematics_);
    this->get_parameter("robot_side", robot_side_);

    RCLCPP_INFO(get_logger(),"SRDF : {%s}",robot_srdf_.c_str());
    RCLCPP_INFO(get_logger(),"DESC : {%s}",robot_description_.c_str());

    RCLCPP_INFO(get_logger(),
    "\n##########################################################"
    "\n##########################################################\n"
    "       ik_link_name='%s' Robot Side : '%s'"
    "\n##########################################################\n"
    "##########################################################",
            ik_link_name_.c_str(), robot_side_.c_str() );

    pub_js_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_topic_, rclcpp::QoS(rclcpp::KeepLast(1)));

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/tracker/tracker_states", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&PoseToJointStateIKNode::onPose, this, std::placeholders::_1));

    sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/system_"+robot_side_+"/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&PoseToJointStateIKNode::onJointCallback, this, std::placeholders::_1));
  }

private:
  void onJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::string s = "[ ";
    for(int i=0;i < 6;i++)
    {
      current_joint[i].store(msg->position[i], std::memory_order_release);   // convert to radian
      s += std::to_string(current_joint[i]) + ", ";
    }
    s += "]";
  }

  void onPose(const geometry_msgs::msg::PoseArray & msg)
  {
    geometry_msgs::msg::PoseStamped target;
    if(msg.poses.empty())
    {
      RCLCPP_INFO(get_logger(),"Pose data empty");
      return;
    }
    target.header = msg.header;        // 원본 frame 유지
    
    if(robot_side_ == "left")
    {
      target.pose = msg.poses[0];        // left는  첫 번째 pose(left)만 사용
    }
    else if(robot_side_ == "right")
    {
      target.pose = msg.poses[1];  // right는 두 번째 pose(right)
    }
    else
    {
      return;
    }

    static bool inited = false;
    static moveit::core::RobotModelPtr robot_model;
    static std::shared_ptr<moveit::core::RobotState> robot_state;
    static const moveit::core::JointModelGroup* jmg = nullptr;
    static std::shared_ptr<robot_model_loader::RobotModelLoader> loader;

    static ContinuousQuat prev_target_quat;

    if (!inited)
    {
      loader = std::make_shared<robot_model_loader::RobotModelLoader>(
          this->shared_from_this(), "robot_description");

      robot_model = loader->getModel();
      if (!robot_model)
      {
        RCLCPP_ERROR(get_logger(), "RobotModel load failed. Check robot_description.");
        return;
      }

      robot_state = std::make_shared<moveit::core::RobotState>(robot_model);

      jmg = robot_model->getJointModelGroup(group_name_.c_str());
      if (!jmg)
      {
        RCLCPP_ERROR(get_logger(), "JointModelGroup not found: %s", group_name_.c_str());
        return;
      }

      inited = true;
    }

    target.pose.orientation = makeContinuousQuaternion(target.pose.orientation, prev_target_quat);

    std::vector<double> initial_joint;
    initial_joint.resize(6);

    std::string t = "[ ";
    for(int i = 0; i < 6; i++)
    {
      initial_joint[i] = static_cast<double>(current_joint[i].load(std::memory_order_acquire) * M_PI / 180.0);
      t += std::to_string(initial_joint[i] * 180.0 / 3.141592) + ", ";
    }
    t += "]";

    robot_state->setJointGroupPositions(jmg, initial_joint);
    robot_state->update();

    const double timeout_sec = 0.05;         // IK 탐색 시간 50ms
    const std::string& ik_link = ik_link_name_; // 예: "tool0"

    bool ok = robot_state->setFromIK(
        jmg,          // 어떤 arm group에 대해 IK를 풀지
        target.pose,  // 목표 pose
        ik_link,      // 어떤 end-effector link를 맞출지
        timeout_sec); // 허용 탐색 시간

    if (!ok)
    {
      RCLCPP_WARN(get_logger(), "Local IK failed (setFromIK).");
      publish_ok.store(false, std::memory_order_release);
      return;
    }

    std::vector<double> res;
    robot_state->copyJointGroupPositions(jmg, res);

    if (res.size() != initial_joint.size())
    {
      RCLCPP_WARN(get_logger(),
                  "IK result size mismatch. seed size=%zu, result size=%zu",
                  initial_joint.size(), res.size());
      publish_ok.store(false, std::memory_order_release);
      return;
    }


    publish_ok.store(true, std::memory_order_release);

    sensor_msgs::msg::JointState out;
    out.header.stamp = this->get_clock()->now();
    out.name = {"j1","j2","j3","j4","j5","j6"};
    out.position.resize(res.size());

    for (size_t i = 0; i < res.size(); ++i)
    {
      out.position[i] = angles::normalize_angle(res[i]) * 180.0 / 3.141592;  // rad >> deg  변환
    }

    if(publish_ok.load(std::memory_order_acquire))
    {
      pub_js_->publish(out);
      // RCLCPP_INFO(get_logger(),"Send ");
    }
  }

  void onPoseAdm(const geometry_msgs::msg::PoseArray & msg)
  {
    geometry_msgs::msg::PoseStamped target;
    if(msg.poses.empty())
    {
      RCLCPP_INFO(get_logger(),"Pose data empty");
      return;
    }
    target.header = msg.header;        // 원본 frame 유지
    
    if(robot_side_ == "left")
    {
      target.pose = msg.poses[0];        // left는  첫 번째 pose(left)만 사용
    }
    else if(robot_side_ == "right")
    {
      target.pose = msg.poses[1];  // right는 두 번째 pose(right)
    }
    else
    {
      return;
    }

    static bool inited = false;
    static moveit::core::RobotModelPtr robot_model;
    static std::shared_ptr<moveit::core::RobotState> robot_state;
    static const moveit::core::JointModelGroup* jmg = nullptr;
    static std::shared_ptr<robot_model_loader::RobotModelLoader> loader;

    static ContinuousQuat prev_target_quat;

    if (!inited)
    {
      loader = std::make_shared<robot_model_loader::RobotModelLoader>(
          this->shared_from_this(), "robot_description");

      robot_model = loader->getModel();
      if (!robot_model)
      {
        RCLCPP_ERROR(get_logger(), "RobotModel load failed. Check robot_description.");
        return;
      }

      robot_state = std::make_shared<moveit::core::RobotState>(robot_model);

      jmg = robot_model->getJointModelGroup(group_name_.c_str());
      if (!jmg)
      {
        RCLCPP_ERROR(get_logger(), "JointModelGroup not found: %s", group_name_.c_str());
        return;
      }

      inited = true;
    }

    target.pose.orientation = makeContinuousQuaternion(target.pose.orientation, prev_target_quat);

    std::vector<double> initial_joint;
    initial_joint.resize(6);

    std::string t = "[ ";
    for(int i = 0; i < 6; i++)
    {
      initial_joint[i] = static_cast<double>(current_joint[i].load(std::memory_order_acquire) * M_PI / 180.0);
      t += std::to_string(initial_joint[i] * 180.0 / 3.141592) + ", ";
    }
    t += "]";

    const double timeout_sec = 0.05;         // IK 탐색 시간 50ms
    const std::string& ik_link = ik_link_name_; // 예: "tool0"

    robot_state->setJointGroupPositions(jmg, initial_joint);
    robot_state->update();

    // FK 로 포즈 추출
    const Eigen::Isometry3d& T = robot_state->getGlobalLinkTransform(ik_link_name_);

    // tracker pose + Quat >> matrix 변환
    Eigen::Isometry3d target_ = getTargetMatrix(target);

    // Jacobian 계산 Pseudo Inverse 계산
    const int cols = static_cast<int>(jmg->getVariableCount());
    
    Eigen::MatrixXd J;
    J.resize(6, cols);
    J.setZero();

    Eigen::Vector3d ref_tcp(0.0, 0.0, 0.0);
    bool ok = robot_state->getJacobian( jmg,
                              robot_state->getLinkModel(ik_link_name_),
                              ref_tcp,
                              J );

    Eigen::MatrixXd J_pinv = pseudoInverse(J);

    Eigen::Vector6d delta_x;

    delta_x.head<3>() = (target_.translation() - T.translation());
    delta_x.tail<3>() = -common_math::getPhi(T.linear(), target_.linear());

    RCLCPP_INFO(get_logger(), "delta : {%lf}, {%lf}, {%lf}, {%lf}, {%lf}, {%lf}",
                delta_x[0],delta_x[1],delta_x[2],delta_x[3],delta_x[4],delta_x[5]);

    if (!ok)
    {
      RCLCPP_WARN(get_logger(), "Get Jacobian Failed");
      publish_ok.store(false, std::memory_order_release);
      return;
    }

    publish_ok.store(true, std::memory_order_release);

    sensor_msgs::msg::JointState out;


    // out.header.stamp = this->get_clock()->now();
    // out.name = {"j1","j2","j3","j4","j5","j6"};
    // out.position.resize(res.size());

    // for (size_t i = 0; i < res.size(); ++i)
    // {
    //   out.position[i] = angles::normalize_angle(res[i]) * 180.0 / 3.141592;  // rad >> deg  변환
    // }

    // if(publish_ok.load(std::memory_order_acquire))
    // {
    //   pub_js_->publish(out);
    // }
  }

  Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J)
  {
    double lambda = 1e-4;

    Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(J.rows(), J.rows());

    Eigen::MatrixXd J_pinv = 
      J.transpose() * (J * J.transpose() + lambda * lambda * I).inverse();

    return J_pinv;
  }

  Eigen::Isometry3d getTargetMatrix(geometry_msgs::msg::PoseStamped msg)
  { 
    Eigen::Quaternion q(msg.pose.orientation.w,
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z);

    Eigen::Vector3d l(msg.pose.position.x,
                      msg.pose.position.y,
                      msg.pose.position.z);

    Eigen::Isometry3d res = Eigen::Isometry3d::Identity();

    res.linear() = q.toRotationMatrix();
    res.translation() = l;

    return res;
  }

  Eigen::Vector6d calcAddmittance()
  {
    Eigen::Vector6d result;

    return result;
  }

  static geometry_msgs::msg::Quaternion makeContinuousQuaternion(
    const geometry_msgs::msg::Quaternion& q_in,
    ContinuousQuat& q_prev)
  {
      tf2::Quaternion q(q_in.x, q_in.y, q_in.z, q_in.w);

      if (q.length2() < 1e-12)
      {
          tf2::Quaternion q_id(0.0, 0.0, 0.0, 1.0);
          geometry_msgs::msg::Quaternion out;
          out = tf2::toMsg(q_id);
          return out;
      }

      q.normalize();
      if (q_prev.initialized)
      {
          const double dot =
              q.x() * q_prev.x +
              q.y() * q_prev.y +
              q.z() * q_prev.z +
              q.w() * q_prev.w;
          if (dot < 0.0)
          {
              q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
          }
      }

      q_prev.x = q.x();
      q_prev.y = q.y();
      q_prev.z = q.z();
      q_prev.w = q.w();
      q_prev.initialized = true;

      geometry_msgs::msg::Quaternion out;
      out = tf2::toMsg(q);
      return out;
  }

  //vars
  std::atomic<float> current_joint[6] = {0.0, 0.0, 0.0, 0, 0, 0};

  std::atomic<float> previous_pose[6];
  std::atomic<bool> publish_ok{false};

  // params
  std::string group_name_;
  std::string ik_link_name_;
  std::string joint_topic_;
  std::string current_joint_topic_;
  std::string robot_description_;
  std::string robot_srdf_;
  std::string robot_kinematics_;
  std::string robot_side_;
  
  // ROS entities
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToJointStateIKNode>());
  rclcpp::shutdown();
  return 0;
}