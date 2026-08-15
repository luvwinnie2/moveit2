// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/behaviors/wait_for_duration.hpp>

namespace moveit2_extended::behaviors
{

WaitForDuration::WaitForDuration(const std::string& name, const NodeConfig& config,
                                 BehaviorContextPtr shared_resources)
  : SharedResourcesNode<BT::StatefulActionNode>(name, config, std::move(shared_resources))
{
}

BT::PortsList WaitForDuration::providedPorts()
{
  return {
    BT::InputPort<double>("duration", 1.0, "seconds to wait"),
  };
}

BtStatus WaitForDuration::onStart()
{
  const double seconds = getInputOr<double>("duration", 1.0);
  if (seconds <= 0.0)
  {
    return BtStatus::SUCCESS;
  }
  deadline_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
  return BtStatus::RUNNING;
}

BtStatus WaitForDuration::onRunning()
{
  return std::chrono::steady_clock::now() >= deadline_ ? BtStatus::SUCCESS : BtStatus::RUNNING;
}

void WaitForDuration::onHalted()
{
  // Reset so a Repeat that halts and restarts this waits the full duration again rather than
  // finishing immediately off the previous deadline.
  deadline_ = std::chrono::steady_clock::time_point{};
}

}  // namespace moveit2_extended::behaviors
