#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <fstream>
#include <memory>
#include <string>

#include "common_action_interface.h"

class DualHdrUpdater;  // forward declaration

template <typename ActionT>
class ActionServerBase : public CommonActionInterface
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using Goal       = typename ActionT::Goal;
  using Result     = typename ActionT::Result;

protected:
  std::string action_name_;
  rclcpp::Time start_time_;
  rclcpp::Node::SharedPtr node_;

  std::atomic_bool control_running_{false};
  std::shared_ptr<GoalHandle> active_goal_handle_{nullptr};

  bool is_initialized_{false};
  bool timer_reset_{true};

  typename rclcpp_action::Server<ActionT>::SharedPtr server_;

protected:
  // goal accept/reject
  virtual rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal) = 0;

  // cancel
  virtual rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle) = 0;

  // accept 이후 실행 트리거
  virtual void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) = 0;

public:
  ActionServerBase(const std::string & name, const rclcpp::Node::SharedPtr & node)
  : action_name_(name), node_(node)
  {}

  virtual ~ActionServerBase() = default;

  void init()
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    server_ = rclcpp_action::create_server<ActionT>(
      node_,
      action_name_,
      std::bind(&ActionServerBase::handleGoal, this, _1, _2),
      std::bind(&ActionServerBase::handleCancel, this, _1),
      std::bind(&ActionServerBase::handleAccepted, this, _1));
  }

  virtual void signalAbort(bool /*is_aborted*/)
  {
    abortActiveGoal();
  }

protected:
  // goal 저장 (mutex 없음)
  void setActiveGoal(const std::shared_ptr<GoalHandle> & gh)
  {
    active_goal_handle_ = gh;
    start_time_ = node_->now();    
  }

  std::shared_ptr<GoalHandle> getActiveGoal() const
  {
    return active_goal_handle_;
  }

  void clearActiveGoal()
  {
    active_goal_handle_.reset();
  }


  virtual void setSucceeded(const std::shared_ptr<Result> & result = std::make_shared<Result>())
  {
    auto gh = getActiveGoal();
    if (!gh) return;

    if (gh->is_active()) {
      gh->succeed(result);
    }
    clearActiveGoal();
  }

  virtual void setAborted(const std::shared_ptr<Result> & result = std::make_shared<Result>())
  {
    auto gh = getActiveGoal();
    if (!gh) return;

    if (gh->is_active()) {
      gh->abort(result);
    }
    clearActiveGoal();
  }

  virtual void setCanceled(const std::shared_ptr<Result> & result = std::make_shared<Result>())
  {
    auto gh = getActiveGoal();
    if (!gh) return;

    if (gh->is_active()) {
      gh->canceled(result);
    }
    clearActiveGoal();
  }

  void abortActiveGoal()
  {
    setAborted(std::make_shared<Result>());
  }
};