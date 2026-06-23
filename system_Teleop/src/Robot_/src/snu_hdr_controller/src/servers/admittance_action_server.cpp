#include "servers/admittance_action_server.h"

template <typename ActionT>
AdmittanceActionServer<ActionT>::AdmittanceActionServer(const std::string &name, const rclcpp::Node::SharedPtr &node): Base(name, node)      
{
    this->init();
    RCLCPP_INFO(this->node_->get_logger(), "[%s] AdmittanceActionServer constructed", name.c_str());
}

template <typename ActionT>
rclcpp_action::GoalResponse AdmittanceActionServer<ActionT>::handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const Goal> goal)
{
    // Validate input
    RCLCPP_INFO(this->node_->get_logger(), "handleGoal called");

    const int len_arms = static_cast<int>(goal->arm_names.size());
    if (len_arms <= 0)
    {
        RCLCPP_WARN(this->node_->get_logger(), "[AdmittanceActionServer] Empty arm_names. Rejecting.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (this->control_running_.load())
    {
        RCLCPP_WARN(this->node_->get_logger(), "[AdmittanceActionServer] Control is already running. Rejecting new goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename ActionT>
rclcpp_action::CancelResponse AdmittanceActionServer<ActionT>::handleCancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    // 현재 active goal만 취소 허용(정책)
    auto active = this->getActiveGoal();
    if (!active || active.get() != goal_handle.get())
    {
        return rclcpp_action::CancelResponse::REJECT;
    }

    RCLCPP_INFO(this->node_->get_logger(), "[AdmittanceActionServer] Cancel requested.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename ActionT>
void AdmittanceActionServer<ActionT>::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    goal_ = goal_handle->get_goal();
    this->setActiveGoal(goal_handle);

    const int len_arms = static_cast<int>(goal_->arm_names.size());

    for (int arm_idx = 0; arm_idx < len_arms; arm_idx++)
    {
        auto & arm_name = goal_->arm_names[arm_idx];

        active_arms_[arm_name] = true;

        AdmittanceController::Param p;
        p.mass = Eigen::Map<const Eigen::Vector6d>(goal_->mass.data());
        p.stiff = Eigen::Map<const Eigen::Vector6d>(goal_->stiff.data());
        p.adm_axis = Eigen::Map<const Eigen::Vector6d>(goal_->adm_axis.data());
        p.zeta = Eigen::Map<const Eigen::Vector6d>(goal_->zeta.data());
        p.dt = goal_->adm_dt;    
        admittance_[arm_name].initialize(p);
    }

    this->control_running_ = true;
    this->start_time_ = this->node_->now();
    this->timer_reset_ = true;

    RCLCPP_INFO(this->node_->get_logger(), "[AdmittanceActionServer] Admittance goal is accepted.");

}

template <typename ActionT>
bool AdmittanceActionServer<ActionT>::compute(DualHdrUpdater &model)
{
    time_ = this->node_->now();

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
            q_admit_[arm_name] = model.q_init_[arm_name];
            x_ee_d_[arm_name] = Eigen::Isometry3d::Identity();
        }
    }

    for (const auto& [arm_name, active] : active_arms_)
    {
        Eigen::Vector6d q_desired;
        Eigen::Isometry3d x_ee;
        Eigen::Vector6d v_ee;
        Eigen::Vector6d delta_x;
        Eigen::Vector6d x_dot_admit;
        Eigen::Vector6d q_admit, q_dot_admit;
        Eigen::Matrix3d R_ref;

        q_desired.setZero();

        if(active)
        {
            x_ee = model.x_ee_[arm_name]; // w.r.t. ee-frame
            v_ee = model.v_ee_[arm_name];

            // adjustment
            R_ref <<  0,  0, -1,
                      0, -1,  0,
                     -1,  0,  0;

            delta_x.head<3>() = (x_ee_d_[arm_name].translation() - x_ee.translation());
            delta_x.tail<3>() = -common_math::getPhi(x_ee.linear(), x_ee_d_[arm_name].linear());
            // delta_x.head<3>() = x_ee.translation() - x_ee_d_[arm_name].translation();
            // delta_x.tail<3>() = -common_math::getPhi(x_ee_d_[arm_name].linear(), x_ee.linear());
            admittance_[arm_name].compute(delta_x, v_ee, model.f_ext_ee_[arm_name]);   // ee frame          
            x_dot_admit = admittance_[arm_name].getAdmittance(R_ref); // w.r.t base frame 

            q_dot_admit = model.J_pinv_[arm_name]*x_dot_admit;

            q_admit_[arm_name] += q_dot_admit*admittance_[arm_name].getDT();

            q_desired = q_admit_[arm_name];
            // q_desired = q_init_[arm_name];

            // std::cout<<arm_name<<" : \n";            
            // std::cout<<"  q_admit_ : "<< q_admit_[arm_name].transpose()*180/M_PI<<"\n";
            // std::cout<<"x_ee : "<< delta_x.head<3>().transpose()<<"\n";
            // std::cout<<"ft : "<< model.f_ext_[arm_name].transpose()<<"\n";
            // std::cout<<"  q_dot_admit : "<< q_dot_admit.transpose()<<"\n";
            // std::cout<<"  q_desired : "<< q_desired.transpose()*180/M_PI<<"\n";
            // std::cout<<"  q_curr : "<< model.q_[arm_name].transpose()<<"\n";

            // debug_q<<q_admit_[arm_name].transpose()*180/M_PI<<" ";
            // // debug_q<<(q_admit - model.q_[arm_name]).transpose()<<" ";
            // debug_qd<<q_dot_admit.transpose()<<" ";

            // debug_ft << model.f_ext_ee_[arm_name].transpose()<<" ";
        }
        else
        {
            q_desired = q_init_[arm_name];
        }

        model.setPosition(q_desired, arm_name);
        // std::cout<<arm_name<<" : \n";            
        // std::cout<<"  q_desired : "<< q_desired.transpose()*180/M_PI<<"\n";

    }
    
    // std::cout<<"left : "<<model.f_ext_["left"].transpose()<<"\n";
    // std::cout<<"right: "<<model.f_ext_["right"].transpose()<<"\n";
    // std::cout<<"----------------------------------------------------"<<"\n";

    // debug_ft<<"\n";
    // debug_q << "\n";
    // debug_qd<<"\n";
    // if (run_time > duration_)
    // {
    //     this->control_running_ = false;
    //     this->is_initialized_ = false;
    //     auto result = std::make_shared<typename ActionT::Result>();
    //     this->setSucceeded(result);
    //     RCLCPP_INFO(this->node_->get_logger(), "[AdmittanceActionServer] Admittance goal completed.");
    // }

    // std::cout<<"run_time : "<< run_time << "/" << duration_<<std::endl;

    double run_time = time_.seconds() - this->start_time_.seconds();
    // if(run_time >= 0.1) // 100ms
    // {
    //     this->timer_reset_ = true;
    //     this->start_time_ = this->node_->now();
    // }

    return true;
}

template <typename ActionT>
bool AdmittanceActionServer<ActionT>::timerReset()
{
    bool result = this->timer_reset_;
    this->timer_reset_ = false;
    return result;
}


template class AdmittanceActionServer<snu_hdr_msgs::action::Admittance>;