// Copyright 2026 Leow Chee Siang. Apache-2.0.
#include <moveit2_extended_core/tree_introspection.hpp>

#include <tinyxml2.h>

#include <algorithm>
#include <functional>
#include <set>

namespace moveit2_extended
{
namespace msgs = moveit2_extended_msgs::msg;

namespace
{
/** Attributes that are structural rather than ports, so they must not be reported as unknown. */
const std::set<std::string> kReservedAttributes = { "ID", "name" };

/** Elements that are part of the tree file format rather than nodes. */
const std::set<std::string> kNonNodeElements = { "root", "BehaviorTree", "TreeNodesModel", "include",
                                                 "Action", "Condition", "Control", "Decorator", "SubTree" };

/** Walk every node element inside every <BehaviorTree>. */
void forEachNodeElement(const tinyxml2::XMLDocument& doc,
                        const std::function<void(const tinyxml2::XMLElement&)>& visit)
{
  const auto* root = doc.FirstChildElement("root");
  if (!root)
  {
    return;
  }
  std::function<void(const tinyxml2::XMLElement*)> recurse = [&](const tinyxml2::XMLElement* element) {
    for (const auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
    {
      const std::string tag = child->Name() ? child->Name() : "";
      if (kNonNodeElements.count(tag) == 0)
      {
        visit(*child);
      }
      recurse(child);
    }
  };
  for (const auto* tree = root->FirstChildElement("BehaviorTree"); tree;
       tree = tree->NextSiblingElement("BehaviorTree"))
  {
    recurse(tree);
  }
}

/** The registration ID an element refers to. For <SubTree ID="X"/> style elements the ID attribute
 *  carries it; otherwise the tag itself is the ID. */
std::string registrationIdOf(const tinyxml2::XMLElement& element)
{
  const char* id = element.Attribute("ID");
  if (id && *id)
  {
    return id;
  }
  return element.Name() ? element.Name() : "";
}
}  // namespace

uint8_t statusToMsg(BtStatus status)
{
  switch (status)
  {
    case BtStatus::IDLE:
      return msgs::BehaviorStatus::IDLE;
    case BtStatus::RUNNING:
      return msgs::BehaviorStatus::RUNNING;
    case BtStatus::SUCCESS:
      return msgs::BehaviorStatus::SUCCESS;
    case BtStatus::FAILURE:
      return msgs::BehaviorStatus::FAILURE;
  }
  return msgs::BehaviorStatus::IDLE;
}

uint8_t nodeTypeToMsg(BtNodeType type)
{
  switch (type)
  {
    case BtNodeType::ACTION:
      return msgs::TreeNodeInfo::ACTION;
    case BtNodeType::CONDITION:
      return msgs::TreeNodeInfo::CONDITION;
    case BtNodeType::CONTROL:
      return msgs::TreeNodeInfo::CONTROL;
    case BtNodeType::DECORATOR:
      return msgs::TreeNodeInfo::DECORATOR;
    case BtNodeType::SUBTREE:
      return msgs::TreeNodeInfo::SUBTREE;
    case BtNodeType::UNDEFINED:
      break;
  }
  return msgs::TreeNodeInfo::UNDEFINED;
}

msgs::TreeStructure describeTree(const BtTree& tree, const std::string& objective_name, const std::string& xml,
                                 const std::string& instance_id)
{
  msgs::TreeStructure out;
  out.objective_name = objective_name;
  out.tree_instance_id = instance_id;
  out.xml = xml;

  const BT::TreeNode* root = tree.rootNode();
  if (!root)
  {
    return out;
  }

  // Depth-first with an explicit parent, so the message can be laid out without a second pass.
  std::function<void(const BT::TreeNode*, uint16_t)> visit = [&](const BT::TreeNode* node, uint16_t parent_uid) {
    if (!node)
    {
      return;
    }
    msgs::TreeNodeInfo info;
    info.uid = node->UID();
    info.parent_uid = parent_uid;
    info.instance_name = node->name();
    info.registration_name = node->registrationName();
    info.node_type = nodeTypeToMsg(node->type());

    std::vector<const BT::TreeNode*> children;
    if (const auto* control = dynamic_cast<const BT::ControlNode*>(node))
    {
      for (const auto* child : control->children())
      {
        children.push_back(child);
      }
    }
    else if (const auto* decorator = dynamic_cast<const BT::DecoratorNode*>(node))
    {
      if (decorator->child())
      {
        children.push_back(decorator->child());
      }
    }
    for (const auto* child : children)
    {
      info.children_uids.push_back(child->UID());
    }

    out.nodes.push_back(info);
    for (const auto* child : children)
    {
      visit(child, node->UID());
    }
  };
  visit(root, 0);
  return out;
}

bool firstFailureIn(const std::vector<msgs::BehaviorStatus>& events, msgs::BehaviorStatus& out)
{
  for (const auto& event : events)
  {
    if (event.current_status == msgs::BehaviorStatus::FAILURE)
    {
      out = event;
      return true;
    }
  }
  return false;
}

namespace
{
/** Deepest node carrying `status` that has no descendant carrying it. */
const BT::TreeNode* deepestWithStatus(const BtTree& tree, BtStatus status)
{
  const BT::TreeNode* root = tree.rootNode();
  if (!root)
  {
    return nullptr;
  }

  const BT::TreeNode* found = nullptr;
  // Returns true when this subtree contains the status anywhere.
  std::function<bool(const BT::TreeNode*)> visit = [&](const BT::TreeNode* node) -> bool {
    if (!node)
    {
      return false;
    }
    std::vector<const BT::TreeNode*> children;
    if (const auto* control = dynamic_cast<const BT::ControlNode*>(node))
    {
      for (const auto* child : control->children())
      {
        children.push_back(child);
      }
    }
    else if (const auto* decorator = dynamic_cast<const BT::DecoratorNode*>(node))
    {
      if (decorator->child())
      {
        children.push_back(decorator->child());
      }
    }

    bool deeper = false;
    for (const auto* child : children)
    {
      deeper = visit(child) || deeper;
    }
    if (node->status() == status && !deeper && !found)
    {
      found = node;  // carries the status with nothing below it that does: the culprit
      return true;
    }
    return deeper || node->status() == status;
  };
  visit(root);
  return found;
}
}  // namespace

const BT::TreeNode* runningLeaf(const BtTree& tree)
{
  return deepestWithStatus(tree, BtStatus::RUNNING);
}

std::vector<msgs::BehaviorInfo> describeBehaviors(const BtFactory& factory,
                                                  const std::unordered_map<std::string, std::string>& loader_of_behavior,
                                                  bool include_builtin)
{
  std::vector<msgs::BehaviorInfo> out;
  const auto& builtin = factory.builtinNodes();

  for (const auto& kv : factory.manifests())
  {
    const BT::TreeNodeManifest& manifest = kv.second;
    if (!include_builtin && builtin.count(manifest.registration_ID))
    {
      continue;
    }

    msgs::BehaviorInfo info;
    info.registration_name = manifest.registration_ID;
    info.node_type = nodeTypeToMsg(manifest.type);
    info.description = manifest.description;

    const auto loader = loader_of_behavior.find(manifest.registration_ID);
    if (loader != loader_of_behavior.end())
    {
      info.loader_package = loader->second;
    }
    else if (builtin.count(manifest.registration_ID))
    {
      info.loader_package = "behaviortree_cpp_v3";
    }

    for (const auto& port_kv : manifest.ports)
    {
      msgs::PortInfo port;
      port.name = port_kv.first;
      switch (port_kv.second.direction())
      {
        case BT::PortDirection::INPUT:
          port.direction = msgs::PortInfo::INPUT;
          break;
        case BT::PortDirection::OUTPUT:
          port.direction = msgs::PortInfo::OUTPUT;
          break;
        case BT::PortDirection::INOUT:
          port.direction = msgs::PortInfo::INOUT;
          break;
      }
      port.type_name = port_kv.second.type() ? BT::demangle(*port_kv.second.type()) : "";
      port.default_value = port_kv.second.defaultValue();
      port.description = port_kv.second.description();
      info.ports.push_back(port);
    }
    std::sort(info.ports.begin(), info.ports.end(),
              [](const msgs::PortInfo& a, const msgs::PortInfo& b) { return a.name < b.name; });
    out.push_back(std::move(info));
  }

  std::sort(out.begin(), out.end(),
            [](const msgs::BehaviorInfo& a, const msgs::BehaviorInfo& b) {
              return a.registration_name < b.registration_name;
            });
  return out;
}

std::vector<std::string> missingBehaviorsInXml(const BtFactory& factory, const std::string& xml)
{
  std::vector<std::string> missing;
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
  {
    return missing;  // the XML itself is broken; that is a separate, earlier error
  }

  // A <SubTree ID="X"/> may refer to another tree in the same file rather than to a Behavior.
  std::set<std::string> local_trees;
  for (const auto& id : treeIdsInXml(xml))
  {
    local_trees.insert(id);
  }

  const auto& builders = factory.builders();
  std::set<std::string> seen;
  forEachNodeElement(doc, [&](const tinyxml2::XMLElement& element) {
    const std::string id = registrationIdOf(element);
    if (id.empty() || local_trees.count(id) || builders.count(id) || seen.count(id))
    {
      return;
    }
    seen.insert(id);
    missing.push_back(id);
  });

  std::sort(missing.begin(), missing.end());
  return missing;
}

std::vector<std::string> unknownPortsInXml(const BtFactory& factory, const std::string& xml)
{
  std::vector<std::string> unknown;
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
  {
    return unknown;
  }

  std::set<std::string> local_trees;
  for (const auto& id : treeIdsInXml(xml))
  {
    local_trees.insert(id);
  }

  const auto& manifests = factory.manifests();
  forEachNodeElement(doc, [&](const tinyxml2::XMLElement& element) {
    const std::string id = registrationIdOf(element);
    if (id.empty() || local_trees.count(id))
    {
      return;  // a subtree reference forwards arbitrary keys; not ours to police
    }
    const auto manifest = manifests.find(id);
    if (manifest == manifests.end())
    {
      return;  // reported by missingBehaviorsInXml instead
    }
    const std::string instance = element.Attribute("name") ? element.Attribute("name") : id;
    for (const auto* attr = element.FirstAttribute(); attr; attr = attr->Next())
    {
      const std::string key = attr->Name() ? attr->Name() : "";
      if (key.empty() || kReservedAttributes.count(key) || manifest->second.ports.count(key))
      {
        continue;
      }
      unknown.push_back(instance + "." + key);
    }
  });

  std::sort(unknown.begin(), unknown.end());
  unknown.erase(std::unique(unknown.begin(), unknown.end()), unknown.end());
  return unknown;
}

bool isWellFormedXml(const std::string& xml, std::string& error)
{
  error.clear();
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) == tinyxml2::XML_SUCCESS)
  {
    return true;
  }
  error = doc.ErrorStr() ? doc.ErrorStr() : "XML parse error";
  if (doc.ErrorLineNum() > 0)
  {
    error += " (line " + std::to_string(doc.ErrorLineNum()) + ")";
  }
  return false;
}

std::vector<std::string> referencedBehaviorsInXml(const std::string& xml)
{
  std::vector<std::string> out;
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
  {
    return out;
  }

  std::set<std::string> local_trees;
  for (const auto& id : treeIdsInXml(xml))
  {
    local_trees.insert(id);
  }

  std::set<std::string> ids;
  forEachNodeElement(doc, [&](const tinyxml2::XMLElement& element) {
    const std::string id = registrationIdOf(element);
    if (!id.empty() && local_trees.count(id) == 0)
    {
      ids.insert(id);
    }
  });
  out.assign(ids.begin(), ids.end());
  return out;
}

std::vector<std::string> invalidPortValuesInXml(const BtFactory& factory, const std::string& xml)
{
  std::vector<std::string> invalid;
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
  {
    return invalid;
  }

  std::set<std::string> local_trees;
  for (const auto& id : treeIdsInXml(xml))
  {
    local_trees.insert(id);
  }

  const auto& manifests = factory.manifests();
  forEachNodeElement(doc, [&](const tinyxml2::XMLElement& element) {
    const std::string id = registrationIdOf(element);
    if (id.empty() || local_trees.count(id))
    {
      return;
    }
    const auto manifest = manifests.find(id);
    if (manifest == manifests.end())
    {
      return;  // missingBehaviorsInXml reports this instead
    }
    const std::string instance = element.Attribute("name") ? element.Attribute("name") : id;

    for (const auto* attr = element.FirstAttribute(); attr; attr = attr->Next())
    {
      const std::string key = attr->Name() ? attr->Name() : "";
      const std::string value = attr->Value() ? attr->Value() : "";
      if (key.empty() || kReservedAttributes.count(key))
      {
        continue;
      }
      const auto port = manifest->second.ports.find(key);
      if (port == manifest->second.ports.end())
      {
        continue;  // unknownPortsInXml reports this instead
      }
      // "{key}" is a blackboard reference: its type is only known at run time, so there is nothing
      // to parse here.
      if (value.size() >= 2 && value.front() == '{' && value.back() == '}')
      {
        continue;
      }
      // An output port takes a destination key, not a value.
      if (port->second.direction() == BT::PortDirection::OUTPUT)
      {
        continue;
      }
      // No registered converter means BehaviorTree.CPP would not parse it either; that is the
      // blackboard-only case, and it is legitimate to leave such a port unwired.
      if (!port->second.type())
      {
        continue;
      }
      try
      {
        port->second.parseString(value);
      }
      catch (const std::exception& exc)
      {
        // what() from the standard converters is the name of the function that threw -- "stod",
        // "stoi" -- which tells the person editing the tree nothing whatsoever. Quote what they
        // actually wrote and name the type the port wants; that is the whole of the diagnosis.
        invalid.push_back(instance + "." + key + ": '" + value + "' is not a valid " +
                          BT::demangle(port->second.type()) + " (" + exc.what() + ")");
      }
    }
  });

  std::sort(invalid.begin(), invalid.end());
  invalid.erase(std::unique(invalid.begin(), invalid.end()), invalid.end());
  return invalid;
}

std::vector<std::string> treeIdsInXml(const std::string& xml)
{
  std::vector<std::string> ids;
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
  {
    return ids;
  }
  const auto* root = doc.FirstChildElement("root");
  if (!root)
  {
    return ids;
  }
  for (const auto* tree = root->FirstChildElement("BehaviorTree"); tree;
       tree = tree->NextSiblingElement("BehaviorTree"))
  {
    const char* id = tree->Attribute("ID");
    if (id && *id)
    {
      ids.push_back(id);
    }
  }
  return ids;
}

}  // namespace moveit2_extended
