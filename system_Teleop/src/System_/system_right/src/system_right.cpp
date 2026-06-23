#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <system_interface/msg/inspire_sensor.hpp>
#include <system_interface/msg/frame_aligned_state.hpp>
#include <system_interface/msg/ui_command.hpp>
#include <system_interface/srv/robot_calibration.hpp>

#include <system_interface/srv/detect_object.hpp>
#include <system_interface/msg/start_recording.hpp>

#include <thread>
#include <atomic>
#include <string>
#include <sstream>
#include <vector>

#define deg2rad M_PI / 180.0
#define rad2deg 180.0 / M_PI

using namespace std;

typedef struct FrameAlignedData
{
  int64_t side;

  int64_t  t_ns;

  uint64_t seq;
  uint32_t frame_index;

  uint32_t width;
  uint32_t height;

  float force_torque[5][6];
  float gripper_joint[20];
  float robot_joint[6];
  float robot_pose[6];
}FrameAlignedData;

static inline int64_t now_steady_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

static inline int64_t ros_time_to_ns(int32_t sec, uint32_t nanosec)
{
    return static_cast<int64_t>(sec) * 1000000000LL +
           static_cast<int64_t>(nanosec);
}

class SystemRightNode : public rclcpp::Node
{
public:
  SystemRightNode()
  : Node("data_aligned_node","system_right")
  {
    this->declare_parameter<int>("record_period", 10);
    record_period_ = this->get_parameter("record_period").as_int();

    if(record_period_ <= 10) record_period_ = 10;    // max 100 Hz

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    inspire_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/inspire/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&SystemRightNode::gripperCallback, this, std::placeholders::_1));

    inspire_sensor_sub = this->create_subscription<system_interface::msg::InspireSensor>(
      "/inspire/tactile_sensor", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&SystemRightNode::gripperSensorCallback, this, std::placeholders::_1));

    robot_ft_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/ft_combined", rclcpp::SensorDataQoS(),
      std::bind(&SystemRightNode::robotForceTorqueCallback, this, std::placeholders::_1));

    robot_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      std::bind(&SystemRightNode::robotJointCallback, this, std::placeholders::_1));

    robot_target_sub = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robot/joint_target_deg", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&SystemRightNode::robotTargetJointCallback, this, std::placeholders::_1));

    cali_sub = this->create_subscription<geometry_msgs::msg::Pose>(
      "pose_states", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&SystemRightNode::robotPoseCallback, this, std::placeholders::_1));

    frame_sub = this->create_subscription<std_msgs::msg::Int32>(
      "frame_index", rclcpp::QoS(rclcpp::KeepLast(1)),
      std::bind(&SystemRightNode::frameCallback, this, std::placeholders::_1));

    data_pub = this->create_publisher<system_interface::msg::FrameAlignedState>(
      "frame_aligned_state", rclcpp::QoS(rclcpp::KeepLast(10)));

    ui_sub = this->create_subscription<system_interface::msg::UiCommand>(
      "ui_command", rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&SystemRightNode::on_request_ui, this, std::placeholders::_1));

    video_recorder_pub = this->create_publisher<system_interface::msg::StartRecording>
    ("start_recording", rclcpp::QoS(rclcpp::KeepLast(10)));

    calibrate_cli = this->create_client<system_interface::srv::RobotCalibration>(
      "/tracker/calibration_server"
    );

    detect_cli = this->create_client<system_interface::srv::DetectObject>(
      "detect_object"
    );

    thread_publish = std::thread(&SystemRightNode::data_publish, this, record_period_);
  }

  ~SystemRightNode()
  {
    thread_running.store(false,std::memory_order_release);

    if(thread_publish.joinable())
      thread_publish.join();
  }

private:
  void on_request_ui(const std::shared_ptr<system_interface::msg::UiCommand> req)
  {
    RCLCPP_INFO(this->get_logger(), "command :{%d}, value : {%d}", req->command, req->value);

    if(req->command == 0) // tracker srv
    {
      if(req->value == 2)
      {
        if(is_detected.load(std::memory_order_acquire))
        {
          float msg_pose[6];
          for(int i=0;i<6 ;i++) msg_pose[i] = recv_robot_pose[i].load(std::memory_order_acquire);
          send_tracker_request("Right", msg_pose, req->value);
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "\n################ Detection Failed ####################");
        }
      }
      else
      {
        float msg_pose[6];
        for(int i=0;i<6 ;i++) msg_pose[i] = recv_robot_pose[i].load(std::memory_order_acquire);
        send_tracker_request("Right", msg_pose, req->value);
      }
    }
    else if(req->command == 1) // data command
    {
      if(req->value == 1) 
      {
        if(is_detected.load(std::memory_order_acquire))
        {
          auto current_stamped_data = stamped_data.load(std::memory_order_acquire);

          record_start_ns_ = now_steady_ns();
          current_stamped_data.t_ns = 0;

          current_stamped_data.seq = 1;
          current_stamped_data.frame_index = 1;

          stamped_data.store(current_stamped_data,std::memory_order_release);

          is_start_record.store(true,std::memory_order_release);
        }
      }
      else 
      {
        is_start_record.store(false,std::memory_order_release);
      }
    }
    else if(req->command == 2)  // Vision Data
    {
      if(req->value == 0)       // Detection
      {
        if(!is_detected.load(std::memory_order_acquire))
        {
          send_detect_request();
          RCLCPP_INFO(this->get_logger(),"DETECTION...");
        }
        else
        {
          float msg_pose[6];
          for(int i=0;i<6 ;i++) msg_pose[i] = recv_robot_pose[i].load(std::memory_order_acquire);
          send_tracker_request("Right", msg_pose, 1);
          RCLCPP_WARN(this->get_logger(),"CALIBRATION --- now Robot can move");
        }
      }
      else if(req->value == 1)  // Record
      {
        // system_interface::msg::StartRecording msg;
        // msg.start_record = true;
        // video_recorder_pub->publish(msg);
        if(is_detected.load(std::memory_order_acquire))
        {
          system_interface::msg::StartRecording msg;
          msg.start_record = true;
          video_recorder_pub->publish(msg);
        }
      }
      else if(req->value == 2)  // stop
      {
        is_detected.store(false, std::memory_order_release);
        system_interface::msg::StartRecording msg;
        msg.start_record = false;
        video_recorder_pub->publish(msg);
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "command x");
    }
  }

  void send_tracker_request(const std::string& side, float* robot_pose, int32_t command)
  {
    auto req = std::make_shared<system_interface::srv::RobotCalibration::Request>();
    req->side = side;
    for (int i = 0; i < 3; ++i) 
    {
      req->robot_pose[i] = robot_pose[i]/1000.0;
      req->robot_pose[i+3] = robot_pose[i+3] * deg2rad;
    }
    req->command = command;

    RCLCPP_INFO(this->get_logger(), "req send");

    calibrate_cli->async_send_request(req,
      [this](rclcpp::Client<system_interface::srv::RobotCalibration>::SharedFuture f)
      {
        auto res = f.get();
        RCLCPP_INFO(this->get_logger(),
                    "Response command = %ld, accepted=%ld",
                    res->code, res->accepted);
      });
  }

  void send_detect_request()
  {
    auto req = std::make_shared<system_interface::srv::DetectObject::Request>();

    auto future = detect_cli->async_send_request(req,
    [this](rclcpp::Client<system_interface::srv::DetectObject>::SharedFuture f)
    {
      auto res = f.get();
      RCLCPP_INFO(this->get_logger(),
                  "Response command = %ld",
                  res->result);

      if(res->result == true)
      {
        RCLCPP_INFO(this->get_logger(),
                    "\n\n\n ############################################# \n ############################################# \n\n            SUCCESS Detecting Hook          \n\n ############################################# \n ############################################# \n\n #####       Press AGAIN       ###### \n\n");

          is_detected.store(true, std::memory_order_release);
      }
      else{
        RCLCPP_ERROR(this->get_logger(),
                    "\n\n\n ############################################# \n ############################################# \n\n            FAILED Detecting Hook          \n\n ############################################# \n ############################################# \n\n #####       Press STOP       ###### \n\n");
        is_detected.store(false, std::memory_order_release);
      }
    });
  }

  void gripperCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    float recv_joint[6] = {0,};

    std::string s = "[ ";
    for(int i=0;i < 6;i++)
    {
      recv_joint[i]= msg->position[i];        // radian
      s += std::to_string(msg->position[i+6]) + " "; 
      if(i % 4 == 0 && i !=0) s += " \n ";

      recv_gripper[i].store(recv_joint[i],std::memory_order_release);
      recv_target_gripper[i].store(msg->position[i+6],std::memory_order_release);
    }
    s += "]";

    // RCLCPP_INFO(this->get_logger(), "\nrecv gtarget : %s\n", s.c_str());
  }

  void gripperSensorCallback(const system_interface::msg::InspireSensor::SharedPtr msg)
  {
    // std::ostringstream oss;
    for(int i=0;i<29;i++)
    {
      // oss << msg->tactile_sensor[i] << ", ";
      recv_gripper_sensor[i].store((float)msg->tactile_sensor[i],std::memory_order_release);
    }
    // RCLCPP_INFO(this->get_logger(), "\ngripper sensor : %s\n", oss.str().c_str());
  }

  void robotTargetJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    float recv_joint[6] = {0,};

    std::ostringstream oss;
    for(int i=0;i < 6;i++)
    {
      oss << msg->position[i] * deg2rad << ", ";
      recv_joint[i]= msg->position[i] * deg2rad;   // convert to radian
      recv_robot_target_joint[i].store(recv_joint[i],std::memory_order_release);
    }
    // RCLCPP_INFO(this->get_logger(), "\nrecv target : %s\n", oss.str().c_str());
  }

  void robotJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    float recv_joint[6] = {0,};

    std::string s = "[ ";
    for(int i=0;i < 6;i++)
    {
      recv_joint[i]= msg->position[i] * deg2rad;   // convert to radian
      s += std::to_string(recv_joint[i] * 180 / 3.141592) + ", ";
      recv_robot_joint[i].store(recv_joint[i],std::memory_order_release);
    }
    s += "]";
  }
  
  void robotPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    float pose[6];

    pose[0]= msg->position.x;
    pose[1]= msg->position.y;
    pose[2]= msg->position.z;
    pose[3]= msg->orientation.x;
    pose[4]= msg->orientation.y;
    pose[5]= msg->orientation.z;
    
    std::ostringstream oss;
    oss << "Recv Pose Data : ";
    for(int i=0;i<6;i++)
    {
      // oss << pose[i] << ", ";
      recv_robot_pose[i].store(pose[i],std::memory_order_release);
    }
    // RCLCPP_INFO(get_logger(), oss.str().c_str());
  }

  void robotForceTorqueCallback(const std_msgs::msg::Float64MultiArray msg)
  {
    for(int i=0;i<6;i++)
    {
      recv_robot_force_torque[i].store(msg.data[i],std::memory_order_release);
    }
  }

  void frameCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    auto current_stamped_data = stamped_data.load(std::memory_order_acquire);
    current_stamped_data.frame_index = msg->data;   // 추후 프레임 데이터 받는 파트
    stamped_data.store(current_stamped_data,std::memory_order_release);
  }

  void data_publish(int period)
  {
    while(true)
    {
      if(!thread_running.load(std::memory_order_acquire))
      {
        RCLCPP_INFO(this->get_logger(), "Thread Break");
        break;
      }

      if(is_start_record.load(std::memory_order_acquire))
      {
        system_interface::msg::FrameAlignedState msg;
        auto current_stamped_data = stamped_data.load(std::memory_order_acquire);

        msg.side = "right";

        const int64_t current_time_ns = now_steady_ns();
        msg.t_ns = current_time_ns - record_start_ns_;
        msg.seq = current_stamped_data.seq++;
        msg.frame_index = current_stamped_data.frame_index;   // 추후 프레임 데이터 받는 파트

        stamped_data.store(current_stamped_data,std::memory_order_release);

        msg.width = 32;
        msg.height = 32;

        for(int i=0;i<6;i++)  // inspire  // append ft sensor
        {
          msg.gripper_joint[i] = recv_gripper[i].load(std::memory_order_acquire) * deg2rad;
          msg.target_gripper_joint[i] = recv_target_gripper[i].load(std::memory_order_acquire)/10.0 * deg2rad;
          
          for(int j=0;j<5;j++)
          {
            msg.gripper_sensor[5*i+j] = recv_gripper_sensor[5*i+j].load(std::memory_order_acquire);
          }
        }

        for(int i=0;i<6;i++)
        {
          msg.robot_ft[i] = recv_robot_force_torque[i].load(std::memory_order_acquire);
          msg.target_robot_joint[i] = recv_robot_target_joint[i].load(std::memory_order_acquire);
          msg.robot_joint[i] = recv_robot_joint[i].load(std::memory_order_acquire);
          msg.robot_pose[i] = recv_robot_pose[i].load(std::memory_order_acquire);
        }

        data_pub->publish(msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(period));
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr inspire_sub;
  rclcpp::Subscription<system_interface::msg::InspireSensor>::SharedPtr inspire_sensor_sub;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_sub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr robot_ft_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_target_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr frame_sub;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cali_sub;
  rclcpp::Subscription<system_interface::msg::UiCommand>::SharedPtr ui_sub;

  rclcpp::Publisher<system_interface::msg::FrameAlignedState>::SharedPtr data_pub;
  rclcpp::Publisher<system_interface::msg::StartRecording>::SharedPtr video_recorder_pub;

  rclcpp::Client<system_interface::srv::RobotCalibration>::SharedPtr calibrate_cli;
  rclcpp::Client<system_interface::srv::DetectObject>::SharedPtr detect_cli;

  int record_period_;

  int64_t record_start_ns_ = 0;

  std::atomic<float> recv_target_gripper[6]{0,};
  std::atomic<float> recv_gripper[6]{0,};
  std::atomic<float> recv_gripper_sensor[30]{0,};
  std::atomic<float> recv_robot_force_torque[6]{0,};
  std::atomic<float> recv_robot_target_joint[6]{0,};
  std::atomic<float> recv_robot_joint[6]{0,};
  std::atomic<float> recv_robot_pose[6]{0,};

  std::atomic<bool> thread_running{true};
  std::atomic<bool> is_start_record {false};
  std::atomic<bool> is_detected{false};
  std::atomic<FrameAlignedData> stamped_data;

  std::thread thread_publish;

  rclcpp::TimerBase::SharedPtr timer_record_; // 타이머 객체를 멤버로 보관해야 유지됨

  float target_joint[6];
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemRightNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}