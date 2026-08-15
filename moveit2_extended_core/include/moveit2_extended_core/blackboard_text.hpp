// Copyright 2026 Leow Chee Siang. Apache-2.0.
#pragma once

#include <moveit2_extended_core/bt_compat.hpp>

#include <string>

namespace moveit2_extended
{

/** Render a blackboard entry as text for a human to read.
 *
 *  Strings and numbers come back as themselves. `bool` is spelled true/false rather than 1/0,
 *  because BT stores bool as an integer and "the carrier is feasible: 0" reads as a count.
 *  Anything else -- a pose, a trajectory -- has no useful one-line form, so it is rendered as
 *  `<type>` rather than throwing: a log line is not worth failing a Behavior over. */
std::string anyToText(const BT::Any& value);

/** Substitute `{key}` references in `text` with values from `blackboard`.
 *
 *  BT.CPP only resolves a port whose value is *entirely* one reference: `message="{reason}"`
 *  arrives already substituted, but `message="failed because {reason}"` arrives verbatim, braces
 *  and all. That is the form every human-readable message wants, so operator-facing Behaviors do
 *  the substitution themselves with this.
 *
 *  Deliberately conservative:
 *   - An unknown key is left exactly as written. Silently blanking it would turn a typo'd port
 *     name into a message that reads fine and says nothing.
 *   - `{{` and `}}` are literal braces.
 *   - A brace that opens and never closes is literal text. */
std::string expandBlackboardReferences(const BT::Blackboard::Ptr& blackboard, const std::string& text);

}  // namespace moveit2_extended
