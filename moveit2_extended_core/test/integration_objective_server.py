#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""End-to-end checks against a running objective server.

The gtests cover the pieces in isolation; this covers the thing an operator actually touches --
the action and the services, over DDS, with the real plugin loading. Several of the behaviours it
pins were bugs found the first time this was run, so it is worth keeping.

    # terminal 1
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=99 \\
        ros2 launch moveit2_extended_core objective_server.launch.py
    # terminal 2
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=99 \\
        python3 install/moveit2_extended_core/share/moveit2_extended_core/test/integration_objective_server.py

Exits non-zero if anything fails, so it can be used as a gate.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from moveit2_extended_msgs.action import ExecuteObjective
from moveit2_extended_msgs.srv import (
    ListBehaviors,
    ListObjectives,
    SaveObjectiveXml,
    ValidateObjectiveXml,
)

RESULT = ExecuteObjective.Result

FAILING_XML = (
    '<root main_tree_to_execute="F"><BehaviorTree ID="F">'
    '<Sequence name="outer">'
    '<LogMessage message="before the failure"/>'
    '<CheckBlackboardValue name="the_culprit" value="a" equals="b"/>'
    "</Sequence></BehaviorTree></root>"
)


def _string(value: str) -> ParameterValue:
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)


class Checks:
    def __init__(self, node: Node) -> None:
        self.node = node
        self.failures: list[str] = []
        self.action = ActionClient(node, ExecuteObjective, "/objective_server/execute_objective")
        if not self.action.wait_for_server(timeout_sec=15.0):
            raise SystemExit("no objective server on /objective_server/execute_objective")

    def check(self, ok: bool, what: str, detail: str = "") -> None:
        print(f"  {'PASS' if ok else 'FAIL'}  {what}{(' -- ' + detail) if detail else ''}")
        if not ok:
            self.failures.append(what)

    def call(self, srv_type, name: str, request):
        client = self.node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=10.0):
            raise SystemExit(f"service {name} never appeared")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        return future.result()

    def run_objective(self, goal: ExecuteObjective.Goal, cancel_after: float | None = None):
        send = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None, 0.0
        result_future = handle.get_result_async()
        started = time.time()
        if cancel_after is not None:
            time.sleep(cancel_after)
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=10.0)
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=60.0)
        wrapped = result_future.result()
        return (wrapped.result if wrapped else None), time.time() - started


def main() -> int:
    rclpy.init()
    node = Node("integration_objective_server")
    checks = Checks(node)

    print("\n[1] a successful objective")
    goal = ExecuteObjective.Goal()
    goal.objective_name = "SmokeTest"
    goal.parameters = [
        Parameter(name="caller", value=_string("cli")),
        Parameter(name="wait_seconds", value=_string("0.2")),
    ]
    goal.export_keys = ["caller"]
    result, elapsed = checks.run_objective(goal)
    checks.check(result is not None and result.success, "objective succeeds")
    if result:
        checks.check(result.error_code == RESULT.SUCCESS, "error_code is SUCCESS")
        checks.check(result.tick_count > 1, "the tree was ticked more than once",
                     f"{result.tick_count} ticks")
        # A string parameter read through a double port: this is what lets the server accept
        # parameters for ports it has never heard of.
        checks.check(0.15 < elapsed < 5.0, "the wait_seconds string reached a double port",
                     f"{elapsed:.2f}s for a 0.2s wait")
        exported = {p.name: p.value.string_value for p in result.outputs}
        checks.check(exported.get("caller") == "cli", "export_keys returns blackboard values",
                     str(exported))

    print("\n[2] a failing objective names the Behavior that failed")
    # The tree forgets: a Sequence halts its children on failure and tickRoot resets the root, so
    # this can only work if the server captured the failing tick's status transitions.
    goal = ExecuteObjective.Goal()
    goal.objective_xml = FAILING_XML
    result, _ = checks.run_objective(goal)
    checks.check(result is not None and not result.success, "objective fails")
    if result:
        checks.check(result.error_code == RESULT.BEHAVIOR_FAILURE, "error_code is BEHAVIOR_FAILURE")
        checks.check(result.failed_behavior_name == "the_culprit",
                     "the deepest failing Behavior is named, not the Sequence above it",
                     f"got '{result.failed_behavior_name}'")
        checks.check(result.failed_behavior_registration == "CheckBlackboardValue",
                     "its registration name is reported too")

    print("\n[3] an unknown objective")
    goal = ExecuteObjective.Goal()
    goal.objective_name = "NoSuchObjective"
    result, _ = checks.run_objective(goal)
    checks.check(result is not None and result.error_code == RESULT.UNKNOWN_OBJECTIVE,
                 "error_code is UNKNOWN_OBJECTIVE")

    print("\n[4] an objective referencing a Behavior nobody registered")
    goal = ExecuteObjective.Goal()
    goal.objective_xml = (
        '<root main_tree_to_execute="M"><BehaviorTree ID="M">'
        '<NotARealBehavior name="x"/></BehaviorTree></root>'
    )
    result, _ = checks.run_objective(goal)
    checks.check(result is not None and result.error_code == RESULT.MISSING_BEHAVIOR,
                 "error_code is MISSING_BEHAVIOR")
    if result:
        checks.check("NotARealBehavior" in result.error_message,
                     "the missing Behavior is named in the message", result.error_message)

    print("\n[5] cancelling a running objective actually halts it")
    goal = ExecuteObjective.Goal()
    goal.objective_name = "SmokeTest"
    goal.parameters = [
        Parameter(name="caller", value=_string("cli")),
        Parameter(name="wait_seconds", value=_string("30.0")),
    ]
    result, elapsed = checks.run_objective(goal, cancel_after=0.5)
    checks.check(result is not None and result.error_code == RESULT.CANCELED,
                 "error_code is CANCELED")
    checks.check(elapsed < 10.0, "it stopped promptly instead of running the full 30 s",
                 f"{elapsed:.2f}s")

    print("\n[6] the editor's node palette")
    response = checks.call(ListBehaviors, "/objective_server/list_behaviors", ListBehaviors.Request())
    names = [b.registration_name for b in response.behaviors]
    checks.check("LogMessage" in names, "our Behaviors are listed", f"{len(names)} behaviors")
    checks.check("Sequence" in names, "BehaviorTree.CPP built-ins are listed too")
    log_message = next((b for b in response.behaviors if b.registration_name == "LogMessage"), None)
    checks.check(log_message is not None and {p.name for p in log_message.ports} >= {"message", "level"},
                 "a Behavior's ports are reported, so the editor can offer them")
    checks.check(log_message is not None and log_message.loader_package == "moveit2_extended_core",
                 "the providing package is reported, so an ID collision is diagnosable")
    checks.check(bool(response.tree_nodes_model_xml), "a Groot-compatible TreeNodesModel is produced")

    print("\n[7] validation refuses the three distinct kinds of broken tree")
    good = checks.call(ValidateObjectiveXml, "/objective_server/validate_objective_xml",
                       ValidateObjectiveXml.Request(xml=FAILING_XML))
    checks.check(good.valid, "a well-formed tree validates", good.xml_error)

    broken = checks.call(ValidateObjectiveXml, "/objective_server/validate_objective_xml",
                         ValidateObjectiveXml.Request(xml="<root><BehaviorTree ID=\"A\"><Sequence></root>"))
    checks.check(not broken.valid and bool(broken.xml_error), "malformed XML is rejected with a parser message")

    missing = checks.call(ValidateObjectiveXml, "/objective_server/validate_objective_xml",
                          ValidateObjectiveXml.Request(
                              xml='<root main_tree_to_execute="A"><BehaviorTree ID="A">'
                                  '<Nope name="x"/></BehaviorTree></root>'))
    checks.check(not missing.valid and "Nope" in list(missing.missing_behaviors),
                 "an unregistered Behavior is named")

    typo = checks.call(ValidateObjectiveXml, "/objective_server/validate_objective_xml",
                       ValidateObjectiveXml.Request(
                           xml='<root main_tree_to_execute="A"><BehaviorTree ID="A">'
                               '<LogMessage name="oops" mesage="typo"/></BehaviorTree></root>'))
    checks.check(not typo.valid and "oops.mesage" in list(typo.unknown_ports),
                 "a mistyped port is caught and pointed at",
                 str(list(typo.unknown_ports)))

    print("\n[8] saving refuses to write a tree that would not build")
    refused = checks.call(SaveObjectiveXml, "/objective_server/save_objective_xml",
                          SaveObjectiveXml.Request(
                              name="ShouldNotBeWritten",
                              xml='<root main_tree_to_execute="A"><BehaviorTree ID="A">'
                                  '<Nope name="x"/></BehaviorTree></root>',
                              file_path="/tmp/should_not_be_written.xml",
                              reload=False))
    checks.check(not refused.success, "a tree with a missing Behavior is not written to disk",
                 refused.message)
    import os
    checks.check(not os.path.exists("/tmp/should_not_be_written.xml"),
                 "and no file was created")

    print("\n[9] listing objectives")
    listed = checks.call(ListObjectives, "/objective_server/list_objectives", ListObjectives.Request())
    smoke = next((o for o in listed.objectives if o.name == "SmokeTest"), None)
    checks.check(smoke is not None, "SmokeTest is listed")
    if smoke:
        checks.check(bool(smoke.description), "its sidecar description came through")
        checks.check("caller" in list(smoke.required_parameters), "its required parameters came through")
        checks.check(not list(smoke.missing_behaviors), "it reports no missing Behaviors")

    print()
    if checks.failures:
        print(f"FAILED: {len(checks.failures)} check(s)")
        for failure in checks.failures:
            print(f"  - {failure}")
    else:
        print("all checks passed")
    rclpy.shutdown()
    return 1 if checks.failures else 0


if __name__ == "__main__":
    sys.exit(main())
