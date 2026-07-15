// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief BT condition: does QuerySemanticContext's first, wide-radius pass
 * already know a candidate (verified or inferred), so Tier-3 can navigate
 * DELIBERATELY to a standoff in front of it (Part A, 2026-07-15) instead of
 * falling back to a blind DriveOnHeading approach?
 *
 * Mirrors responsible_object_matcher.should_trust_supplied_match's exact
 * semantics (verified/inferred -> usable; unknown/empty -> not) so the C++
 * and Python sides of the recovery pipeline agree on what counts as "a
 * candidate worth trusting."
 */
class HasResponsibleObjectCandidate : public BT::ConditionNode
{
public:
  HasResponsibleObjectCandidate(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  static bool hasCandidate(const std::string & match_type);
};

}  // namespace semantic_nav_nav2_plugins
