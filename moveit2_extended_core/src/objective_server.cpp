// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/objective_server.hpp>
#include <moveit2_extended_core/path_utils.hpp>
#include <moveit2_extended_core/status_logger.hpp>
#include <moveit2_extended_core/tree_introspection.hpp>

#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <sstream>

namespace moveit2_extended
{
namespace msgs = moveit2_extended_msgs::msg;
namespace srvs = moveit2_extended_msgs::srv;
namespace fs = std::filesystem;

namespace
{
constexpr const char* kAmentResource = "moveit2_extended_behavior_plugin";

rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).transient_local().reliable();
}
}  // namespace

ObjectiveServer::ObjectiveServer(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("objective_server", options))
{
  declareParameters();
  createContext();
  loadBehaviorLoaders();
  loadObjectives();
  createInterfaces();

  RCLCPP_INFO(node_->get_logger(), "objective server ready: %zu behavior(s), %zu objective(s)",
              factory_->builders().size(), library_.size());
}

ObjectiveServer::~ObjectiveServer()
{
  if (worker_.joinable())
  {
    worker_.join();
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ObjectiveServer::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

void ObjectiveServer::declareParameters()
{
  node_->declare_parameter<std::vector<std::string>>("objective_directories", std::vector<std::string>{});
  node_->declare_parameter<bool>("objective_directories_recursive", false);
  node_->declare_parameter<std::string>("behavior_loader_discovery", "ament_index");
  node_->declare_parameter<std::vector<std::string>>("behavior_loader_plugins", std::vector<std::string>{});
  node_->declare_parameter<std::vector<std::string>>("behavior_loader_packages_exclude", std::vector<std::string>{});
  node_->declare_parameter<std::vector<std::string>>("behavior_loader_load_order", std::vector<std::string>{});
  node_->declare_parameter<std::vector<std::string>>("behavior_parameter_files", std::vector<std::string>{});
  node_->declare_parameter<int>("tick_period_ms", 20);
  node_->declare_parameter<double>("default_objective_timeout", 0.0);

  node_->declare_parameter<std::string>("default_planning_group", "arm");
  node_->declare_parameter<std::string>("robot_description_param", "robot_description");
  node_->declare_parameter<bool>("start_planning_scene_monitor", true);
  node_->declare_parameter<double>("moveit_connect_timeout", 10.0);

  tick_period_ = std::chrono::milliseconds(std::max<int64_t>(1, node_->get_parameter("tick_period_ms").as_int()));
  default_timeout_ = node_->get_parameter("default_objective_timeout").as_double();
}

void ObjectiveServer::createContext()
{
  BehaviorContextConfig config;
  config.default_planning_group = node_->get_parameter("default_planning_group").as_string();
  config.robot_description_param = node_->get_parameter("robot_description_param").as_string();
  config.start_planning_scene_monitor = node_->get_parameter("start_planning_scene_monitor").as_bool();
  config.moveit_connect_timeout = node_->get_parameter("moveit_connect_timeout").as_double();

  std::string error;
  const auto files = node_->get_parameter("behavior_parameter_files").as_string_array();
  BehaviorParameterMap parameters = BehaviorParameterMap::fromFiles(files, &error);
  if (!error.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "behavior parameters: %s", error.c_str());
  }

  context_ = std::make_shared<BehaviorContext>(node_, std::move(config), std::move(parameters));
  factory_ = std::make_unique<BtFactory>();
}

void ObjectiveServer::loadBehaviorLoaders()
{
  class_loader_ = std::make_shared<pluginlib::ClassLoader<BehaviorLoaderBase>>(
      "moveit2_extended_core", "moveit2_extended::BehaviorLoaderBase");

  const std::string discovery = node_->get_parameter("behavior_loader_discovery").as_string();
  const auto excluded = node_->get_parameter("behavior_loader_packages_exclude").as_string_array();
  const auto load_order = node_->get_parameter("behavior_loader_load_order").as_string_array();

  // package -> classes
  std::map<std::string, std::vector<std::string>> to_load;

  if (discovery == "ament_index" || discovery == "both")
  {
    // Each Behavior package registers the ament resource "moveit2_extended_behavior_plugin"
    // pointing at its behavior_plugin.yaml. We load exactly the classes those files list --
    // never everything pluginlib can see, which would dlopen loaders meant for other robots and
    // let one throwing constructor take the whole server down.
    for (const auto& kv : ament_index_cpp::get_resources(kAmentResource))
    {
      const std::string& package = kv.first;
      if (std::find(excluded.begin(), excluded.end(), package) != excluded.end())
      {
        RCLCPP_INFO(node_->get_logger(), "excluding behavior package '%s' by configuration", package.c_str());
        continue;
      }

      std::string content;
      std::string prefix;
      if (!ament_index_cpp::get_resource(kAmentResource, package, content, &prefix))
      {
        continue;
      }
      const std::string relative(content.begin(), content.end());
      const std::string manifest = (fs::path(prefix) / relative).string();

      try
      {
        const YAML::Node root = YAML::LoadFile(manifest);
        const YAML::Node plugins = root["behavior_loader_plugins"];
        if (!plugins || !plugins.IsMap())
        {
          RCLCPP_WARN(node_->get_logger(), "%s has no behavior_loader_plugins map", manifest.c_str());
          continue;
        }
        for (const auto& entry : plugins)
        {
          const std::string manifest_package = entry.first.as<std::string>();
          for (const auto& cls : entry.second)
          {
            to_load[manifest_package].push_back(cls.as<std::string>());
          }
        }
      }
      catch (const std::exception& exc)
      {
        RCLCPP_ERROR(node_->get_logger(), "cannot read %s: %s", manifest.c_str(), exc.what());
      }
    }
  }

  if (discovery == "explicit" || discovery == "both")
  {
    for (const auto& cls : node_->get_parameter("behavior_loader_plugins").as_string_array())
    {
      to_load["<explicit>"].push_back(cls);
    }
  }

  // Deterministic order: packages named in behavior_loader_load_order first, then the rest
  // alphabetically. Order matters because the first registration of an ID wins and the second
  // throws, so without this which package wins would vary between machines.
  std::vector<std::string> packages;
  packages.reserve(to_load.size());
  for (const auto& kv : to_load)
  {
    packages.push_back(kv.first);
  }
  std::sort(packages.begin(), packages.end());
  std::stable_sort(packages.begin(), packages.end(), [&load_order](const std::string& a, const std::string& b) {
    const auto ia = std::find(load_order.begin(), load_order.end(), a);
    const auto ib = std::find(load_order.begin(), load_order.end(), b);
    return std::distance(load_order.begin(), ia) < std::distance(load_order.begin(), ib);
  });

  for (const auto& package : packages)
  {
    for (const auto& class_name : to_load[package])
    {
      try
      {
        auto loader = class_loader_->createSharedInstance(class_name);
        loader->setSource(package, class_name);

        const auto before = factory_->builders();
        loader->registerBehaviors(*factory_, context_);

        for (const auto& kv : factory_->builders())
        {
          if (before.count(kv.first) == 0)
          {
            loader_of_behavior_[kv.first] = package;
          }
        }
        for (const auto& kv : loader->behaviorDescriptions())
        {
          factory_->addDescriptionToManifest(kv.first, kv.second);
        }
        loaders_.push_back(std::move(loader));
        RCLCPP_INFO(node_->get_logger(), "loaded behaviors from %s (%s)", package.c_str(), class_name.c_str());
      }
      catch (const std::exception& exc)
      {
        // Keep going: one package's collision or bad constructor must not cost every other
        // package's Behaviors.
        RCLCPP_ERROR(node_->get_logger(), "behavior loader '%s' from package '%s' failed: %s", class_name.c_str(),
                     package.c_str(), exc.what());
      }
    }
  }
}

void ObjectiveServer::loadObjectives()
{
  std::lock_guard<std::mutex> lock(library_mutex_);
  const auto directories = node_->get_parameter("objective_directories").as_string_array();
  const bool recursive = node_->get_parameter("objective_directories_recursive").as_bool();

  std::vector<std::string> errors;
  library_.load(*factory_, directories, recursive, &errors);
  for (const auto& error : errors)
  {
    RCLCPP_ERROR(node_->get_logger(), "objective: %s", error.c_str());
  }
}

void ObjectiveServer::createInterfaces()
{
  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<ExecuteObjective>(
      node_, "~/execute_objective", std::bind(&ObjectiveServer::handleGoal, this, _1, _2),
      std::bind(&ObjectiveServer::handleCancel, this, _1), std::bind(&ObjectiveServer::handleAccepted, this, _1));

  log_pub_ = node_->create_publisher<msgs::BehaviorTreeLog>("~/behavior_tree_log", rclcpp::QoS(10));
  state_pub_ = node_->create_publisher<msgs::ObjectiveState>("~/objective_state", latchedQos());
  structure_pub_ = node_->create_publisher<msgs::TreeStructure>("~/tree_structure", latchedQos());

  list_objectives_srv_ = node_->create_service<srvs::ListObjectives>(
      "~/list_objectives", std::bind(&ObjectiveServer::onListObjectives, this, _1, _2));
  get_objective_srv_ = node_->create_service<srvs::GetObjective>(
      "~/get_objective", std::bind(&ObjectiveServer::onGetObjective, this, _1, _2));
  list_behaviors_srv_ = node_->create_service<srvs::ListBehaviors>(
      "~/list_behaviors", std::bind(&ObjectiveServer::onListBehaviors, this, _1, _2));
  reload_srv_ = node_->create_service<srvs::ReloadObjectives>(
      "~/reload_objectives", std::bind(&ObjectiveServer::onReloadObjectives, this, _1, _2));
  validate_srv_ = node_->create_service<srvs::ValidateObjectiveXml>(
      "~/validate_objective_xml", std::bind(&ObjectiveServer::onValidateObjectiveXml, this, _1, _2));
  save_srv_ = node_->create_service<srvs::SaveObjectiveXml>(
      "~/save_objective_xml", std::bind(&ObjectiveServer::onSaveObjectiveXml, this, _1, _2));

  publishState(msgs::ObjectiveState::IDLE, "", "", "", 0);
}

std::vector<std::string> ObjectiveServer::registeredBehaviors() const
{
  std::vector<std::string> out;
  for (const auto& kv : factory_->builders())
  {
    out.push_back(kv.first);
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<std::string> ObjectiveServer::registeredObjectives() const
{
  std::lock_guard<std::mutex> lock(library_mutex_);
  return library_.names();
}

// ---------------------------------------------------------------------------------------------
// action
// ---------------------------------------------------------------------------------------------

rclcpp_action::GoalResponse ObjectiveServer::handleGoal(const rclcpp_action::GoalUUID&,
                                                        std::shared_ptr<const ExecuteObjective::Goal> goal)
{
  if (busy_.load())
  {
    RCLCPP_WARN(node_->get_logger(), "rejecting '%s': an objective is already running",
                goal->objective_name.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->objective_name.empty() && goal->objective_xml.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "rejecting a goal with neither objective_name nor objective_xml");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ObjectiveServer::handleCancel(const std::shared_ptr<GoalHandle>&)
{
  // Accepted here; the worker notices is_canceling() between ticks and halts the tree from its own
  // thread. Halting from this callback's thread would race the tick.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ObjectiveServer::handleAccepted(const std::shared_ptr<GoalHandle>& goal_handle)
{
  if (worker_.joinable())
  {
    worker_.join();  // the previous run has finished; busy_ guards against overlap
  }
  busy_.store(true);
  worker_ = std::thread([this, goal_handle]() { runObjective(goal_handle); });
}

void ObjectiveServer::runObjective(const std::shared_ptr<GoalHandle>& goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ExecuteObjective::Result>();
  const std::string instance_id = "obj_" + std::to_string(++instance_counter_);
  const auto started_at = std::chrono::steady_clock::now();

  const auto finish = [&](uint8_t code, const std::string& message, uint8_t state) {
    result->error_code = code;
    result->success = (code == ExecuteObjective::Result::SUCCESS);
    result->error_message = message;
    result->execution_time =
        rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
                                                                             started_at));
    publishState(state, goal->objective_name, instance_id, "", result->tick_count);
    busy_.store(false);
  };

  std::string xml = goal->objective_xml;
  std::string tree_to_run = goal->objective_name;
  {
    std::lock_guard<std::mutex> lock(library_mutex_);
    if (xml.empty())
    {
      const auto* entry = library_.find(goal->objective_name);
      if (!entry)
      {
        finish(ExecuteObjective::Result::UNKNOWN_OBJECTIVE, "no objective named '" + goal->objective_name + "'",
               msgs::ObjectiveState::FAILED);
        goal_handle->abort(result);
        return;
      }
      xml = entry->xml;
    }
  }

  // Checked BEFORE createTree() so the failure names what is missing, instead of surfacing a raw
  // BehaviorTree.CPP exception that says only that something went wrong.
  result->missing_behaviors = missingBehaviorsInXml(*factory_, xml);
  if (!result->missing_behaviors.empty())
  {
    std::ostringstream os;
    os << "objective references " << result->missing_behaviors.size() << " unregistered behavior(s): ";
    for (size_t i = 0; i < result->missing_behaviors.size(); ++i)
    {
      os << (i ? ", " : "") << result->missing_behaviors[i];
    }
    finish(ExecuteObjective::Result::MISSING_BEHAVIOR, os.str(), msgs::ObjectiveState::FAILED);
    goal_handle->abort(result);
    return;
  }

  BtTree tree;
  try
  {
    auto blackboard = BtBlackboard::create();
    seedBlackboard(blackboard, goal->parameters);
    if (!goal->objective_xml.empty())
    {
      tree = factory_->createTreeFromText(xml, blackboard);
    }
    else
    {
      tree = factory_->createTree(tree_to_run, blackboard);
    }
  }
  catch (const std::exception& exc)
  {
    finish(ExecuteObjective::Result::XML_PARSE_ERROR, exc.what(), msgs::ObjectiveState::FAILED);
    goal_handle->abort(result);
    return;
  }

  structure_pub_->publish(describeTree(tree, goal->objective_name, xml, instance_id));

  ObjectiveStatusLogger logger(tree, node_->get_clock());
  const double timeout = goal->timeout > 0.0 ? goal->timeout : default_timeout_;

  BtStatus status = BtStatus::RUNNING;
  uint8_t final_code = ExecuteObjective::Result::SUCCESS;
  std::string final_message;
  std::vector<msgs::BehaviorStatus> last_tick_events;
  auto last_feedback = std::chrono::steady_clock::now();

  publishState(msgs::ObjectiveState::RUNNING, goal->objective_name, instance_id, "", 0);

  while (rclcpp::ok())
  {
    if (goal_handle->is_canceling())
    {
      haltTree(tree);
      final_code = ExecuteObjective::Result::CANCELED;
      final_message = "canceled by request";
      break;
    }
    if (timeout > 0.0)
    {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
      if (elapsed > timeout)
      {
        haltTree(tree);
        final_code = ExecuteObjective::Result::TIMEOUT;
        final_message = "timed out after " + std::to_string(timeout) + " s";
        break;
      }
    }

    try
    {
      status = tickOnce(tree);
    }
    catch (const std::exception& exc)
    {
      haltTree(tree);
      final_code = ExecuteObjective::Result::BEHAVIOR_EXCEPTION;
      final_message = exc.what();
      break;
    }
    ++result->tick_count;

    auto events = logger.drain();
    // Kept from the tick that actually failed. The tree cannot be asked afterwards: a Sequence
    // halts its children when one fails, resetting them to IDLE, and tickRoot() resets the root
    // too -- so by the time this loop sees FAILURE, nothing in the tree remembers who caused it.
    if (status == BtStatus::FAILURE)
    {
      last_tick_events = events;
    }
    const auto* running = runningLeaf(tree);
    const std::string current = running ? running->name() : "";

    const bool due = std::chrono::steady_clock::now() - last_feedback > std::chrono::milliseconds(500);
    if (!events.empty() || due)
    {
      auto feedback = std::make_shared<ExecuteObjective::Feedback>();
      feedback->tree_instance_id = instance_id;
      feedback->tree_status = statusToMsg(status);
      feedback->changed = events;
      feedback->current_behavior = current;
      feedback->elapsed = rclcpp::Duration(
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - started_at));
      feedback->tick_count = result->tick_count;
      goal_handle->publish_feedback(feedback);

      if (!events.empty())
      {
        msgs::BehaviorTreeLog log;
        log.stamp = node_->get_clock()->now();
        log.tree_instance_id = instance_id;
        log.objective_name = goal->objective_name;
        log.event_log = std::move(events);
        log_pub_->publish(log);
      }
      publishState(msgs::ObjectiveState::RUNNING, goal->objective_name, instance_id, current, result->tick_count);
      last_feedback = std::chrono::steady_clock::now();
    }

    if (status != BtStatus::RUNNING)
    {
      break;
    }
    sleepBetweenTicks(tree, tick_period_);
  }

  if (final_code == ExecuteObjective::Result::SUCCESS && status == BtStatus::FAILURE)
  {
    final_code = ExecuteObjective::Result::BEHAVIOR_FAILURE;
    msgs::BehaviorStatus culprit;
    if (firstFailureIn(last_tick_events, culprit))
    {
      result->failed_behavior_name = culprit.instance_name;
      result->failed_behavior_registration = culprit.registration_name;
      result->failed_behavior_uid = culprit.uid;
      final_message = "'" + culprit.instance_name + "' (" + culprit.registration_name + ") returned FAILURE";
    }
    else
    {
      final_message = "the tree returned FAILURE";
    }
  }

  // Export requested blackboard keys before the tree (and its blackboard) go out of scope.
  if (!goal->export_keys.empty() && tree.rootBlackboard())
  {
    const auto blackboard = tree.rootBlackboard();
    std::lock_guard<std::mutex> lock(blackboard->entryMutex());
    for (const auto& key : goal->export_keys)
    {
      const BT::Any* any = blackboard->getAny(key);
      if (!any || any->empty())
      {
        continue;
      }
      rcl_interfaces::msg::Parameter parameter;
      parameter.name = key;
      parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      try
      {
        parameter.value.string_value = any->cast<std::string>();
      }
      catch (const std::exception&)
      {
        // Not string-convertible; report the type so the caller knows why it is not here.
        parameter.value.string_value = "<" + BT::demangle(any->type()) + ">";
      }
      result->outputs.push_back(std::move(parameter));
    }
  }

  const uint8_t state = final_code == ExecuteObjective::Result::SUCCESS       ? msgs::ObjectiveState::SUCCEEDED
                        : final_code == ExecuteObjective::Result::CANCELED   ? msgs::ObjectiveState::CANCELED
                                                                            : msgs::ObjectiveState::FAILED;
  finish(final_code, final_message, state);

  if (final_code == ExecuteObjective::Result::SUCCESS)
  {
    goal_handle->succeed(result);
  }
  else if (final_code == ExecuteObjective::Result::CANCELED)
  {
    goal_handle->canceled(result);
  }
  else
  {
    goal_handle->abort(result);
  }
}

// ---------------------------------------------------------------------------------------------
// services
// ---------------------------------------------------------------------------------------------

void ObjectiveServer::onListObjectives(const std::shared_ptr<srvs::ListObjectives::Request> request,
                                       std::shared_ptr<srvs::ListObjectives::Response> response)
{
  std::lock_guard<std::mutex> lock(library_mutex_);
  for (const auto& name : library_.names())
  {
    if (!request->name_filter.empty() && name.find(request->name_filter) == std::string::npos)
    {
      continue;
    }
    if (const auto* entry = library_.find(name))
    {
      response->objectives.push_back(library_.describe(*factory_, *entry));
    }
  }
}

void ObjectiveServer::onGetObjective(const std::shared_ptr<srvs::GetObjective::Request> request,
                                     std::shared_ptr<srvs::GetObjective::Response> response)
{
  std::lock_guard<std::mutex> lock(library_mutex_);
  const auto* entry = library_.find(request->name);
  response->found = (entry != nullptr);
  if (entry)
  {
    response->xml = entry->xml;
    response->info = library_.describe(*factory_, *entry);
  }
}

void ObjectiveServer::onListBehaviors(const std::shared_ptr<srvs::ListBehaviors::Request>,
                                      std::shared_ptr<srvs::ListBehaviors::Response> response)
{
  response->behaviors = describeBehaviors(*factory_, loader_of_behavior_);
  response->tree_nodes_model_xml = BT::writeTreeNodesModelXML(*factory_);
}

void ObjectiveServer::onReloadObjectives(const std::shared_ptr<srvs::ReloadObjectives::Request>,
                                         std::shared_ptr<srvs::ReloadObjectives::Response> response)
{
  if (busy_.load())
  {
    // The running Tree holds pointers into the registered definitions; clearing them under it
    // would be a use-after-free.
    response->success = false;
    response->message = "an objective is running; reload refused";
    std::lock_guard<std::mutex> lock(library_mutex_);
    response->num_objectives = static_cast<uint32_t>(library_.size());
    return;
  }

  std::lock_guard<std::mutex> lock(library_mutex_);
  library_.clear(*factory_);
  const auto directories = node_->get_parameter("objective_directories").as_string_array();
  const bool recursive = node_->get_parameter("objective_directories_recursive").as_bool();

  std::vector<std::string> errors;
  const bool ok = library_.load(*factory_, directories, recursive, &errors);
  response->success = ok;
  response->num_objectives = static_cast<uint32_t>(library_.size());
  if (!ok)
  {
    std::ostringstream os;
    for (size_t i = 0; i < errors.size(); ++i)
    {
      os << (i ? "; " : "") << errors[i];
    }
    response->message = os.str();
  }
  else
  {
    response->message = "reloaded " + std::to_string(library_.size()) + " objective(s)";
  }
}

bool ObjectiveServer::validateXml(const std::string& xml, std::string& xml_error, std::vector<std::string>& missing,
                                  std::vector<std::string>& unknown_ports,
                                  std::vector<std::string>& invalid_values,
                                  std::vector<std::string>& tree_ids) const
{
  xml_error.clear();
  missing.clear();
  unknown_ports.clear();
  invalid_values.clear();
  tree_ids.clear();

  // Order matters. BT::VerifyXML rejects unknown node types as well as malformed XML, so running
  // it first would report "your XML is broken" for a tree whose only problem is an uninstalled
  // Behavior package -- and send the reader hunting for a missing bracket that is not there.
  //
  // So: syntax first, then the missing-Behavior and port checks, and only run VerifyXML for its
  // structural rules once we know every node type is available to it.
  if (!isWellFormedXml(xml, xml_error))
  {
    return false;
  }

  tree_ids = treeIdsInXml(xml);
  missing = missingBehaviorsInXml(*factory_, xml);
  unknown_ports = unknownPortsInXml(*factory_, xml);
  invalid_values = invalidPortValuesInXml(*factory_, xml);
  if (!missing.empty())
  {
    return false;
  }

  std::unordered_map<std::string, BtNodeType> registered;
  for (const auto& kv : factory_->manifests())
  {
    registered[kv.first] = kv.second.type;
  }
  try
  {
    BT::VerifyXML(xml, registered);
  }
  catch (const std::exception& exc)
  {
    xml_error = exc.what();
    return false;
  }

  // Port literals are checked here rather than left to BehaviorTree.CPP, which parses them lazily
  // on the first getInput() -- so a malformed pose builds fine and throws mid-motion.
  return unknown_ports.empty() && invalid_values.empty();
}

void ObjectiveServer::onValidateObjectiveXml(const std::shared_ptr<srvs::ValidateObjectiveXml::Request> request,
                                             std::shared_ptr<srvs::ValidateObjectiveXml::Response> response)
{
  response->valid = validateXml(request->xml, response->xml_error, response->missing_behaviors,
                                response->unknown_ports, response->invalid_port_values, response->tree_ids);
}

void ObjectiveServer::onSaveObjectiveXml(const std::shared_ptr<srvs::SaveObjectiveXml::Request> request,
                                         std::shared_ptr<srvs::SaveObjectiveXml::Response> response)
{
  // Validate first, always. The editor must not be able to leave an Objective on disk that this
  // server then refuses to load.
  std::vector<std::string> tree_ids;
  if (!validateXml(request->xml, response->xml_error, response->missing_behaviors, response->unknown_ports,
                   response->invalid_port_values, tree_ids))
  {
    response->success = false;
    response->message = "refused to save: the tree would not build";
    return;
  }

  std::string path = request->file_path;
  if (path.empty())
  {
    std::lock_guard<std::mutex> lock(library_mutex_);
    const auto* entry = library_.find(request->name);
    if (!entry)
    {
      response->success = false;
      response->message = "no file_path given and no objective named '" + request->name + "' to overwrite";
      return;
    }
    path = entry->file;
  }

  std::string resolve_error;
  const std::string resolved = resolvePackageUri(path, &resolve_error);
  if (resolved.empty())
  {
    response->success = false;
    response->message = resolve_error;
    return;
  }

  // The editor emits normalised XML, so comments and hand formatting do not survive a round trip.
  // Keep the previous contents rather than destroying them silently.
  std::error_code ec;
  if (fs::exists(resolved, ec))
  {
    const std::string backup = resolved + ".bak";
    fs::copy_file(resolved, backup, fs::copy_options::overwrite_existing, ec);
    if (!ec)
    {
      response->backup_path = backup;
    }
    else
    {
      response->success = false;
      response->message = "cannot write the backup " + backup + ": " + ec.message();
      return;
    }
  }

  std::string write_error;
  if (!writeFile(resolved, request->xml, &write_error))
  {
    response->success = false;
    response->message = write_error;
    return;
  }
  response->written_path = resolved;

  if (request->reload && !busy_.load())
  {
    std::lock_guard<std::mutex> lock(library_mutex_);
    library_.clear(*factory_);
    std::vector<std::string> errors;
    library_.load(*factory_, node_->get_parameter("objective_directories").as_string_array(),
                  node_->get_parameter("objective_directories_recursive").as_bool(), &errors);
  }

  response->success = true;
  response->message = "saved";
}

// ---------------------------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------------------------

std::string ObjectiveServer::parameterToString(const rcl_interfaces::msg::ParameterValue& value)
{
  using rcl_interfaces::msg::ParameterType;
  std::ostringstream os;
  switch (value.type)
  {
    case ParameterType::PARAMETER_BOOL:
      return value.bool_value ? "true" : "false";
    case ParameterType::PARAMETER_INTEGER:
      return std::to_string(value.integer_value);
    case ParameterType::PARAMETER_DOUBLE:
      os << value.double_value;
      return os.str();
    case ParameterType::PARAMETER_STRING:
      return value.string_value;
    case ParameterType::PARAMETER_BOOL_ARRAY:
      for (size_t i = 0; i < value.bool_array_value.size(); ++i)
      {
        os << (i ? ";" : "") << (value.bool_array_value[i] ? "true" : "false");
      }
      return os.str();
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      for (size_t i = 0; i < value.integer_array_value.size(); ++i)
      {
        os << (i ? ";" : "") << value.integer_array_value[i];
      }
      return os.str();
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      for (size_t i = 0; i < value.double_array_value.size(); ++i)
      {
        os << (i ? ";" : "") << value.double_array_value[i];
      }
      return os.str();
    case ParameterType::PARAMETER_STRING_ARRAY:
      // Semicolon-separated, which is what BehaviorTree.CPP's own vector converters expect.
      for (size_t i = 0; i < value.string_array_value.size(); ++i)
      {
        os << (i ? ";" : "") << value.string_array_value[i];
      }
      return os.str();
    default:
      return {};
  }
}

void ObjectiveServer::seedBlackboard(const BtBlackboard::Ptr& blackboard,
                                     const std::vector<rcl_interfaces::msg::Parameter>& parameters)
{
  for (const auto& parameter : parameters)
  {
    // Everything goes in as a string on purpose: BehaviorTree.CPP converts a string blackboard
    // entry to the port's declared type when the port is read, so this works for port types this
    // package has never heard of and needs no type registry.
    blackboard->set(parameter.name, parameterToString(parameter.value));
  }
}

void ObjectiveServer::publishState(uint8_t state, const std::string& objective_name, const std::string& instance_id,
                                   const std::string& current_behavior, uint32_t tick_count)
{
  msgs::ObjectiveState message;
  message.stamp = node_->get_clock()->now();
  message.state = state;
  message.objective_name = objective_name;
  message.tree_instance_id = instance_id;
  message.current_behavior = current_behavior;
  message.tick_count = tick_count;
  state_pub_->publish(message);
}

}  // namespace moveit2_extended
