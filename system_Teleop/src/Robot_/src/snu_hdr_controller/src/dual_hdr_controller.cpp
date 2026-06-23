#include "dual_hdr_controller.h"

namespace dual_hdr_controller
{
    DualHdrController::DualHdrController(const std::string urdf_path, const rclcpp::Node::SharedPtr &node) : node_(node), urdf_path_(urdf_path)
    {
        std::cout << "Dual Hdr Controller is ready to be initialized" << std::endl;

        initialize(urdf_path);
        registerServers(node_);

        moveit_interface_ = std::make_shared<MoveItInterface>(node_);

        ft_node_ = std::make_shared<rclcpp::Node>("ft_sensor_node");
        ft_sensor_ = std::make_shared<FTSensorReader>(ft_node_, "/ft_combined");
    }

    void DualHdrController::initialize(const std::string urdf_path)
    {
        model_.initialize(urdf_path);
    }

    void DualHdrController::registerServers(const rclcpp::Node::SharedPtr &node)
    {
        idle_control_server_ = std::make_shared<IdleControlServer>("/snu_hdr_controller/idle_control", node);
        moveit_action_server_ = std::make_shared<MoveitActionServer<moveit_msgs::action::ExecuteTrajectory>>("/execute_trajectory", node);
        joint_move_action_server_ = std::make_shared<JointMoveActionServer<snu_hdr_msgs::action::JointMove>>("/snu_hdr_controller/joint_move_control", node);
        admittance_action_server_ = std::make_shared<AdmittanceActionServer<snu_hdr_msgs::action::Admittance>>("/snu_hdr_controller/admittance_control", node);

        action_servers_.push_back(moveit_action_server_);
        action_servers_.push_back(joint_move_action_server_);
        action_servers_.push_back(admittance_action_server_);
    }

    bool DualHdrController::update(const Eigen::VectorXd &q,
                                   const Eigen::VectorXd &qd)
    {        
        Eigen::VectorXd ft = ft_sensor_->getFT();
        model_.updateModel(q, qd, ft);

        moveit_interface_->publishJointState(q);

        for (auto &as : action_servers_)
        {
            if(as->compute(model_))
            {                   
                timer_reset_ = as->timerReset();         
                return true;
            }
        }

        timer_reset_ = idle_control_server_->timerReset(model_);
        idle_control_server_->compute(model_);
        return true;
    }

    void DualHdrController::validate()
    {
        // for(size_t i = 0; i < model_.arm_names_.size(); i++){
        //     // set the cmd as a current joint angle to stop robot's motion
        //     Eigen::Vector3d x;
        //     x = model_.transform_[model_.arm_names_[i]].translation();
        //     if(x_limit(x[0], 1.5, -1.5) || y_limit(x[1], 1.5, -1.5) || z_limit(x[2], 2.5, 0.5)){
        //         model_.setPosition(model_.q_[model_.arm_names_[i]], model_.arm_names_[i]); 
        //     }

        //     if(selfCollision(x.head<2>(), 1.0))
        //     {
        //         model_.setPosition(model_.q_[model_.arm_names_[i]], model_.arm_names_[i]); 
        //     }
        // }

    }

    Eigen::VectorXd DualHdrController::write()
    {
        return model_.getPosition();
    }

    bool DualHdrController::getTimerReset()
    {
        return timer_reset_;
    }

}