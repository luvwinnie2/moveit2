// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/async_behavior_base.hpp>

namespace moveit2_extended
{

AsyncBehaviorBase::AsyncBehaviorBase(const std::string& name, const NodeConfig& config,
                                     BehaviorContextPtr shared_resources)
  : SharedResourcesNode<BT::StatefulActionNode>(name, config, std::move(shared_resources))
{
}

AsyncBehaviorBase::~AsyncBehaviorBase()
{
  // A worker still running here would use members that are about to be destroyed. Ask it to stop
  // and give it the same grace period a halt would.
  cancel_requested_.store(true, std::memory_order_release);
  if (future_.valid())
  {
    future_.wait_for(halt_join_timeout_);
  }
}

void AsyncBehaviorBase::notifyDone()
{
  emitStateChanged();
}

BtStatus AsyncBehaviorBase::onStart()
{
  cancel_requested_.store(false, std::memory_order_release);

  // Everything that needs the blackboard, the robot model or the planning scene happens here, on
  // the tick thread, before any worker exists to race with it.
  if (!prepare())
  {
    return BtStatus::FAILURE;
  }

  future_ = std::async(std::launch::async, [this]() {
    try
    {
      return doWork();
    }
    catch (const std::exception& exc)
    {
      // An exception on the worker thread would otherwise be re-thrown at .get() on the tick
      // thread and take down the whole tree. Report it as a failure of this Behavior instead.
      RCLCPP_ERROR(getLogger(), "threw: %s", exc.what());
      return BtStatus::FAILURE;
    }
  });
  // Always RUNNING, even if the work is already done: see the class comment.
  return BtStatus::RUNNING;
}

BtStatus AsyncBehaviorBase::onRunning()
{
  if (!future_.valid())
  {
    RCLCPP_ERROR(getLogger(), "onRunning() with no work in flight");
    return BtStatus::FAILURE;
  }
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    return BtStatus::RUNNING;
  }

  const BtStatus result = future_.get();
  publishResults(result);  // tick thread: the only place output ports may be written
  return result;
}

void AsyncBehaviorBase::onHalted()
{
  cancel_requested_.store(true, std::memory_order_release);
  if (!future_.valid())
  {
    return;
  }
  if (future_.wait_for(halt_join_timeout_) != std::future_status::ready)
  {
    // Detaching leaks the worker until it finishes on its own. That is bad, and it is the derived
    // class's contract that was broken -- so say so loudly rather than blocking the tree for ever.
    RCLCPP_ERROR(getLogger(),
                 "worker did not stop within %ld ms of a halt; it is not polling cancelRequested(). "
                 "Leaving it detached.",
                 static_cast<long>(halt_join_timeout_.count()));
    return;
  }
  future_.get();  // drop the result; nobody is waiting for it after a halt
}

}  // namespace moveit2_extended
