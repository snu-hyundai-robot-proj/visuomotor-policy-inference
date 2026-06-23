#include "servers/moveit_action_server.h"

template <typename ActionT>
MoveitActionServer<ActionT>::MoveitActionServer(const std::string &name, const rclcpp::Node::SharedPtr &node): Base(name, node)      
{
    this->init();
    traj_index_ = 1;
    RCLCPP_INFO(this->node_->get_logger(), "[%s] MoveitActionServer constructed", name.c_str());

}

template <typename ActionT>
rclcpp_action::GoalResponse MoveitActionServer<ActionT>::handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Goal> goal)
{

    RCLCPP_INFO(this->node_->get_logger(), "Received MoveitAction goal");

    
    if (this->control_running_.load())
    {
        RCLCPP_WARN(this->node_->get_logger(), "[MoveitActionServer] Control is already running. Rejecting new goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ActionT>
rclcpp_action::CancelResponse MoveitActionServer<ActionT>::handleCancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    // 현재 active goal만 취소 허용(정책)
    auto active = this->getActiveGoal();

    if (!active || active.get() != goal_handle.get())
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    RCLCPP_INFO(this->node_->get_logger(), "[MoveitActionServer] Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT>
void MoveitActionServer<ActionT>::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    goal_ = goal_handle->get_goal();
    this->setActiveGoal(goal_handle);

    RCLCPP_INFO(this->node_->get_logger(), "[MoveitActionServer] Moveit goal accepted.");

    parseTrajectory(goal_->trajectory);

    this->control_running_ = true;
    this->start_time_ = this->node_->now();
    this->timer_reset_ = true;

}

template <typename ActionT>
bool MoveitActionServer<ActionT>::compute(DualHdrUpdater &model)
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
       
        traj_start_time_ = this->node_->now(); // set initial start time         
    }

    if (q_traj_.size() == 1)
    {
        this->control_running_ = false;
        this->is_initialized_ = false;
        auto result = std::make_shared<typename ActionT::Result>();
        this->setSucceeded(result);
        RCLCPP_INFO(this->node_->get_logger(), "[MoveitActionServer] Single-point trajectory completed.");
        return true;
    }

    Eigen::VectorXd q;
    Eigen::VectorXd q_desired;
    Eigen::VectorXd q_interp;
    double t_0, t_f, t_elapsed;

    q.resize(model.n_dof_);
    for(size_t i = 0; i < model.n_arms_; i ++){
        int offset = i * model.n_joints_;
        q.segment(offset, model.n_joints_) = model.q_[model.arm_names_[i]];
    }

    q_desired.resize(model.n_dof_);
    q_desired.setZero();

    t_0 = time_from_start_.at(traj_index_ - 1);    
    t_f = time_from_start_.at(traj_index_);   
    t_elapsed = this->node_->now().seconds() - traj_start_time_.seconds();
 
    q_interp = q_traj_.at(traj_index_);
    Eigen::VectorXd q_tmp = q_traj_.at(traj_index_-1);

    for(int i = 0; i < model.n_dof_; i ++)
    {
        // q_desired[i] = common_math::cubic(t_elapsed, t_0, t_f, q[i], q_interp[i], 0.0, 0.0);
        q_desired[i] = common_math::cubic(t_elapsed, t_0, t_f, q_tmp[i], q_interp[i], 0.0, 0.0);

    }

    for(size_t i = 0; i < model.n_arms_; i ++){
        int offset = i * model.n_joints_;
        Eigen::VectorXd q_cmd;
        q_cmd.resize(model.n_joints_);
        q_cmd = q_desired.segment(offset, model.n_joints_);        
        model.setPosition(q_cmd, model.arm_names_[i]);
    }
    

    // std::cout<< traj_index_ << " / " << q_traj_.size() << "\n"
    // << "t   : " << t_elapsed << "\n"
    // << "t_f : " <<time_from_start_.at(traj_index_)<<"\n"
    // <<"---------------------------------------------------\n";

    if(t_elapsed >= t_f){
        traj_index_++; // to update target        
    }    

    if (traj_index_ >= q_traj_.size())
    {
        this->control_running_ = false;
        this->is_initialized_ = false;
        auto result = std::make_shared<typename ActionT::Result>();
        this->setSucceeded(result);
        RCLCPP_INFO(this->node_->get_logger(), "[MoveitActionServer] Joint move goal completed.");
        RCLCPP_INFO(this->node_->get_logger(), "[MoveitActionServer] Joint move goal completed. Total runtime: %.3f sec / %.3f sec", 
            run_time, time_from_start_.at(time_from_start_.size() - 1));

        traj_index_ = 1;

    }

    // debug_q_traj << q_desired.transpose()<<"\n";
    return true;
}

template <typename ActionT>
void MoveitActionServer<ActionT>::parseTrajectory(const moveit_msgs::msg::RobotTrajectory &traj)
{
    q_traj_.clear();
    time_from_start_.clear();

    const auto &jt = traj.joint_trajectory;

    for (const auto &pt : jt.points)
    {
        Eigen::VectorXd q(pt.positions.size());
        for (size_t i = 0; i < pt.positions.size(); ++i)
            q(i) = pt.positions[i];

        q_traj_.push_back(q);

        double t = pt.time_from_start.sec + 1e-9 * pt.time_from_start.nanosec;

        time_from_start_.push_back(t);
    }

    start_time_ = this->node_->now();
    
}

template <typename ActionT>
bool MoveitActionServer<ActionT>::timerReset()
{
    bool result = this->timer_reset_;
    this->timer_reset_ = false;
    return result;
}


template class MoveitActionServer<moveit_msgs::action::ExecuteTrajectory>;