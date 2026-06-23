#include "servers/idle_action_server.h"

IdleControlServer::IdleControlServer(const std::string &name, const rclcpp::Node::SharedPtr &node) : node_(node), name_(name)
{
    if (!node_)
    {
        throw std::invalid_argument("IdleControlServer: node is nullptr");
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] IdleControlServer constructed", name_.c_str());
}

void IdleControlServer::compute(DualHdrUpdater &model)
{

    if (model.idle_controlled_)
    {
        model.setInitialValues();
        model.idle_controlled_ = false;                
        RCLCPP_INFO(node_->get_logger(), "Reset Idle Controller");
    }

    for(size_t i = 0; i < model.arm_names_.size(); i++){
        
        std::string arm_name;
        Eigen::VectorXd q_desired;

        arm_name = model.arm_names_[i];        
        q_desired = model.q_init_[arm_name];
                
        model.setPosition(q_desired, arm_name);
    }
    
    
}

bool IdleControlServer::timerReset(DualHdrUpdater &model)
{    
    return model.idle_controlled_;
}
