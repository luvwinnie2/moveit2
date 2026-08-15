// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/shared_resources_node.hpp>

#include <atomic>
#include <chrono>
#include <future>

namespace moveit2_extended
{

/** A Behavior whose work is a blocking computation, run on a worker thread so the tick thread
 *  stays responsive and halt() means something.
 *
 *  THREAD RULES -- this is the only class in the project where two threads meet, so the split is
 *  deliberately narrow:
 *
 *    doWork()          runs on a worker thread. MUST NOT touch the blackboard, setOutput(), or any
 *                      other BehaviorTree.CPP method: the tick thread owns those. Must poll
 *                      cancelRequested() often enough to honour a halt.
 *    publishResults()  runs on the TICK thread once doWork() has returned. This is the only place
 *                      a derived class may write output ports.
 *
 *  onStart() always returns RUNNING, even for work that finishes instantly, so a caller always
 *  sees at least one RUNNING tick and Parallel/ReactiveSequence semantics stay predictable. */
class AsyncBehaviorBase : public SharedResourcesNode<BT::StatefulActionNode>
{
public:
  AsyncBehaviorBase(const std::string& name, const NodeConfig& config, BehaviorContextPtr shared_resources);
  ~AsyncBehaviorBase() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

protected:
  /** Gather everything the work needs, on the TICK thread, before the worker starts.
   *
   *  This is where anything that touches the blackboard, the robot model or the planning scene
   *  belongs -- doWork() may touch none of them. Return false to fail the Behavior immediately
   *  without launching a worker; log the reason yourself, since only the caller knows what went
   *  wrong. */
  virtual bool prepare()
  {
    return true;
  }

  /** The work. Runs once, on a worker thread. */
  virtual BtStatus doWork() = 0;

  /** Write output ports here. Runs on the tick thread after doWork() returned. */
  virtual void publishResults(BtStatus /*result*/)
  {
  }

  bool cancelRequested() const
  {
    return cancel_requested_.load(std::memory_order_acquire);
  }

  /** For handing cancellation to a long-running library call that polls a flag of its own. */
  const std::atomic_bool& cancelFlag() const
  {
    return cancel_requested_;
  }

  /** Call from doWork() when it finishes early, so the tree's sleep is cut short instead of
   *  waiting out the rest of the tick period. */
  void notifyDone();

  /** How long onHalted() waits for a worker that is ignoring cancelRequested(). */
  std::chrono::milliseconds halt_join_timeout_{ 2000 };

private:
  BtStatus onStart() final;
  BtStatus onRunning() final;
  void onHalted() final;

  std::future<BtStatus> future_;
  std::atomic_bool cancel_requested_{ false };
};

}  // namespace moveit2_extended
