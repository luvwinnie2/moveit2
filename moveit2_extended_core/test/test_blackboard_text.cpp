// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Message interpolation, tested because the failure mode is silent and user-facing: an operator
// approval prompt that reads "works the carrier to {worst_bend} of its limit" is a question nobody
// can answer, and nothing crashes to tell you.

#include <moveit2_extended_core/blackboard_text.hpp>

#include <gtest/gtest.h>

using moveit2_extended::anyToText;
using moveit2_extended::expandBlackboardReferences;

namespace
{
BT::Blackboard::Ptr makeBlackboard()
{
  auto bb = BT::Blackboard::create();
  bb->set("reason", std::string("carrier bend limit"));
  bb->set("bad_index", 17);
  bb->set("worst_bend", 0.93);
  bb->set("feasible", false);
  return bb;
}
}  // namespace

TEST(BlackboardText, SubstitutesAnEmbeddedReference)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "rejected at waypoint {bad_index}"), "rejected at waypoint 17");
}

TEST(BlackboardText, SubstitutesSeveralInOneLine)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "{reason} at {bad_index}"), "carrier bend limit at 17");
}

/** The whole point of the change: this is the shape every operator-facing message has. */
TEST(BlackboardText, HandlesAReferenceInsideParentheses)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "works the carrier to {worst_bend} of its limit ({reason}). Run it?"),
            "works the carrier to 0.930000 of its limit (carrier bend limit). Run it?");
}

/** BT stores bool as an integer, so without the special case this would read "... : 0". */
TEST(BlackboardText, SpellsBoolRatherThanPrintingZero)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "feasible: {feasible}"), "feasible: false");
}

/** A typo must stay visible. Blanking it produces a message that reads fine and says nothing. */
TEST(BlackboardText, LeavesAnUnknownKeyVerbatim)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "value {no_such_key} here"), "value {no_such_key} here");
}

TEST(BlackboardText, LeavesTextWithoutReferencesAlone)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "nothing to do here"), "nothing to do here");
}

TEST(BlackboardText, DoubledBracesAreLiteral)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "literal {{reason}} stays"), "literal {reason} stays");
}

TEST(BlackboardText, AnUnterminatedBraceIsLiteralRatherThanTruncating)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "half open {reason and more"), "half open {reason and more");
}

/** A key with a space in it is not a reference -- prose in braces must survive. */
TEST(BlackboardText, IgnoresBracesAroundProse)
{
  auto bb = makeBlackboard();
  EXPECT_EQ(expandBlackboardReferences(bb, "see {the manual} for details"), "see {the manual} for details");
}

TEST(BlackboardText, ToleratesANullBlackboard)
{
  EXPECT_EQ(expandBlackboardReferences(nullptr, "value {reason}"), "value {reason}");
}

/** Types with no sensible one-line form must not throw out of a log call. */
TEST(BlackboardText, RendersAnUnprintableTypeAsItsTypeName)
{
  auto bb = BT::Blackboard::create();
  bb->set("pose", std::vector<double>{ 1.0, 2.0 });
  const auto out = expandBlackboardReferences(bb, "pose is {pose}");
  EXPECT_NE(out.find('<'), std::string::npos) << out;
  EXPECT_EQ(out.find("{pose}"), std::string::npos) << out;
}

TEST(BlackboardText, AnyToTextOnAnEmptyAnyDoesNotThrow)
{
  EXPECT_NO_THROW({ EXPECT_EQ(anyToText(BT::Any()), "<empty>"); });
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
