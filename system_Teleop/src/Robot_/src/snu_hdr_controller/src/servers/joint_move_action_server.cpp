#include "servers/joint_move_action_server.h"

template <typename ActionT>
JointMoveActionServer<ActionT>::JointMoveActionServer(const std::string &name, const rclcpp::Node::SharedPtr &node): Base(name, node)      
{
    this->init();
    RCLCPP_INFO(this->node_->get_logger(), "[%s] JointMoveActionServer constructed", name.c_str());

}

template <typename ActionT>
rclcpp_action::GoalResponse JointMoveActionServer<ActionT>::handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Goal> goal)
{
    // Validate input
    RCLCPP_INFO(this->node_->get_logger(), "handleGoal called");

    const int len_arms = static_cast<int>(goal->arm_names.size());
    if (len_arms <= 0)
    {
        RCLCPP_WARN(this->node_->get_logger(), "[JointMoveActionServer] Empty arm_names. Rejecting.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->execution_time <= 0.0)
    {
        RCLCPP_WARN(this->node_->get_logger(), "[JointMoveActionServer] execution_time <= 0. Rejecting.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (this->control_running_.load())
    {
        RCLCPP_WARN(this->node_->get_logger(), "[JointMoveActionServer] Control is already running. Rejecting new goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ActionT>
rclcpp_action::CancelResponse JointMoveActionServer<ActionT>::handleCancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    // 현재 active goal만 취소 허용(정책)
    auto active = this->getActiveGoal();
    if (!active || active.get() != goal_handle.get())
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    RCLCPP_INFO(this->node_->get_logger(), "[JointMoveActionServer] Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT>
void JointMoveActionServer<ActionT>::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    goal_ = goal_handle->get_goal();
    this->setActiveGoal(goal_handle);

    const int len_arms = static_cast<int>(goal_->arm_names.size());
    
    for (int arm_idx = 0; arm_idx < len_arms; arm_idx++)
    {
        auto & arm_name = goal_->arm_names[arm_idx];
        auto & target_pose = goal_->target_q[arm_idx];

        active_arms_[arm_name] = true;

        for(int i = 0; i < 6; i ++){
            q_target_[arm_name](i) = target_pose.position.at(i);
        }
    }

    duration_ = goal_->execution_time;


    this->control_running_ = true;
    this->start_time_ = this->node_->now();
    this->timer_reset_ = true;

    RCLCPP_INFO(this->node_->get_logger(), "[JointMoveActionServer] Joint move goal accepted.");

}

template <typename ActionT>
bool JointMoveActionServer<ActionT>::compute(DualHdrUpdater &model)
{
    double run_time = this->node_->now().seconds() - this->start_time_.seconds();

    auto gh = this->getActiveGoal();
    if (!gh) return false;
    if (!gh->is_active()) return false;
    if (!this->control_running_.load()) return false;

    if (!this->is_initialized_)
    {
        model.setInitialValues();
        this->is_initialized_ = true;        
        model.idle_controlled_ = true;

        for (const auto& [arm_name, active] : active_arms_)
        {
            q_init_[arm_name] = model.q_init_[arm_name];
        }
    }

    for (const auto& [arm_name, active] : active_arms_)
    {
        Eigen::Vector6d q  = model.q_[arm_name];
        Eigen::Vector6d qd = model.qd_[arm_name];        
        Eigen::Vector6d q_desired;

        if (active)
        {
            for (int i = 0; i < model.n_joints_; i++)
            {
                q_desired[i] = common_math::cubic(run_time, 0.0, duration_, q_init_[arm_name][i], q_target_[arm_name][i], 0.0, 0.0);
            }
        }
        else
        {
            q_desired = q_init_[arm_name];
        }

        model.setPosition(q_desired, arm_name);
    }

    if (run_time > duration_)
    {
        this->control_running_ = false;
        this->is_initialized_ = false;
        auto result = std::make_shared<typename ActionT::Result>();
        this->setSucceeded(result);
        RCLCPP_INFO(this->node_->get_logger(), "[JointMoveActionServer] Joint move goal completed.");
    }

    // std::cout<<"run_time : "<< run_time << "/" << duration_<<std::endl;
    return true;
}

template <typename ActionT>
bool JointMoveActionServer<ActionT>::timerReset()
{
    bool result = this->timer_reset_;
    this->timer_reset_ = false;
    return result;
}


template class JointMoveActionServer<snu_hdr_msgs::action::JointMove>;