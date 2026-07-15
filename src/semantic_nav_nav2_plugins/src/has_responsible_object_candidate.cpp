// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/has_responsible_object_candidate.hpp"

namespace semantic_nav_nav2_plugins
{

HasResponsibleObjectCandidate::HasResponsibleObjectCandidate(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
}

bool HasResponsibleObjectCandidate::hasCandidate(const std::string & match_type)
{
  return match_type == "verified" || match_type == "inferred";
}

BT::NodeStatus HasResponsibleObjectCandidate::tick()
{
  std::string match_type;
  getInput("match_type", match_type);
  return hasCandidate(match_type) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList HasResponsibleObjectCandidate::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "match_type", "",
      "responsible_match_type from QuerySemanticContext's first, wide-radius"
      " pass; SUCCESS for verified/inferred, FAILURE for unknown/empty"),
  };
}

}  // namespace semantic_nav_nav2_plugins
