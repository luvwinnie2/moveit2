#!/usr/bin/env python3
# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""The Studio's backend: a ROS node that also serves a web UI.

WHY NOT ROSBRIDGE. The browser only ever needs this project's own services, not arbitrary ROS
access, so a generic bridge would be a much larger attack surface and another package to install
for no gain. A handful of JSON endpoints backed by rclpy is smaller, needs nothing beyond the
standard library, and cannot be used to publish to a topic nobody thought about.

WHAT IT SERVES:
    GET  /                      the UI
    GET  /api/state             everything the UI polls: objective state, tree, tools, waypoints
    GET  /api/behaviors         the node palette for the editor
    GET  /api/objective/<name>  an Objective's XML
    POST /api/objective/<name>  validate and save it
    POST /api/validate          validate XML without saving
    POST /api/run               run an Objective
    POST /api/cancel            cancel the running one
    POST /api/answer            answer a user prompt
    POST /api/tool              fit a tool
    POST /api/waypoint          teach or delete a waypoint

Everything is polled rather than pushed. At the rates a person can read -- a few hertz -- polling
is simpler than a socket, survives a reload, and cannot leave a half-open connection behind.
"""

from __future__ import annotations

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from moveit2_extended_msgs.action import ExecuteObjective
from moveit2_extended_msgs.msg import ObjectiveState, TreeStructure, UserPrompt, UserResponse
from moveit2_extended_msgs.srv import (
    DeleteWaypoint,
    GetActiveTool,
    GetObjective,
    ListBehaviors,
    ListObjectives,
    ListTools,
    ListWaypoints,
    SaveObjectiveXml,
    SaveWaypoint,
    SwitchTool,
    ValidateObjectiveXml,
)
from moveit2_extended_msgs.msg import BehaviorTreeLog


class StudioNode(Node):
    """Holds the ROS side. Every handler runs on the HTTP thread, so anything it touches is either
    immutable or guarded."""

    def __init__(self) -> None:
        # studio_bridge, not studio_server: IsUserAvailable looks for this name to decide whether
        # anybody is watching, and an Objective that blocks on a prompt nobody will see is worse
        # than one that takes the automatic branch.
        super().__init__("studio_bridge")

        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        self._state: dict[str, Any] = {"state": "IDLE", "objective": "", "current_behavior": "", "ticks": 0}
        self._tree: dict[str, Any] = {}
        self._statuses: dict[int, int] = {}
        self._prompt: dict[str, Any] = {}
        self._log: list[str] = []

        self.create_subscription(ObjectiveState, "/objective_server/objective_state",
                                 self._on_state, self._latched(), callback_group=self._group)
        self.create_subscription(TreeStructure, "/objective_server/tree_structure",
                                 self._on_tree, self._latched(), callback_group=self._group)
        self.create_subscription(BehaviorTreeLog, "/objective_server/behavior_tree_log",
                                 self._on_log, 10, callback_group=self._group)
        self.create_subscription(UserPrompt, "/objective/user_prompt",
                                 self._on_prompt, self._latched(), callback_group=self._group)
        self._response_pub = self.create_publisher(UserResponse, "/objective/user_response", 10)

        self._action = ActionClient(self, ExecuteObjective, "/objective_server/execute_objective",
                                    callback_group=self._group)
        self._goal_handle = None

        # NOT self._clients: rclpy's Node already uses that name for its own list of service
        # clients, and assigning a dict over it makes the executor iterate the dict's KEYS --
        # which fails with "'str' object has no attribute '_executor_event'" on the first spin.
        self._service_clients = {
            "list_objectives": self.create_client(ListObjectives, "/objective_server/list_objectives",
                                                  callback_group=self._group),
            "get_objective": self.create_client(GetObjective, "/objective_server/get_objective",
                                                callback_group=self._group),
            "list_behaviors": self.create_client(ListBehaviors, "/objective_server/list_behaviors",
                                                 callback_group=self._group),
            "validate": self.create_client(ValidateObjectiveXml, "/objective_server/validate_objective_xml",
                                           callback_group=self._group),
            "save": self.create_client(SaveObjectiveXml, "/objective_server/save_objective_xml",
                                       callback_group=self._group),
            "list_tools": self.create_client(ListTools, "/tool_manager/list", callback_group=self._group),
            "active_tool": self.create_client(GetActiveTool, "/tool_manager/active", callback_group=self._group),
            "switch_tool": self.create_client(SwitchTool, "/tool_manager/switch", callback_group=self._group),
            "list_waypoints": self.create_client(ListWaypoints, "/arm_waypoint_manager/list",
                                                 callback_group=self._group),
            "save_waypoint": self.create_client(SaveWaypoint, "/arm_waypoint_manager/save",
                                                callback_group=self._group),
            "delete_waypoint": self.create_client(DeleteWaypoint, "/arm_waypoint_manager/delete",
                                                  callback_group=self._group),
        }

    @staticmethod
    def _latched():
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)

    # ---- subscriptions -------------------------------------------------------------------------

    def _on_state(self, message: ObjectiveState) -> None:
        names = {0: "IDLE", 1: "RUNNING", 2: "SUCCEEDED", 3: "FAILED", 4: "CANCELED"}
        with self._lock:
            self._state = {
                "state": names.get(message.state, "?"),
                "objective": message.objective_name,
                "current_behavior": message.current_behavior,
                "ticks": message.tick_count,
                "instance": message.tree_instance_id,
            }

    def _on_tree(self, message: TreeStructure) -> None:
        with self._lock:
            self._tree = {
                "objective": message.objective_name,
                "instance": message.tree_instance_id,
                "xml": message.xml,
                "nodes": [
                    {
                        "uid": n.uid,
                        "parent": n.parent_uid,
                        "name": n.instance_name,
                        "registration": n.registration_name,
                        "type": n.node_type,
                        "children": list(n.children_uids),
                    }
                    for n in message.nodes
                ],
            }
            # A new tree means the previous run's colours are meaningless.
            self._statuses = {}

    def _on_log(self, message: BehaviorTreeLog) -> None:
        with self._lock:
            for event in message.event_log:
                self._statuses[event.uid] = event.current_status
                self._log.append(f"{event.instance_name} -> {self._status_name(event.current_status)}")
            # Bounded: a long run would otherwise grow this without limit.
            self._log = self._log[-200:]

    @staticmethod
    def _status_name(value: int) -> str:
        return {0: "IDLE", 1: "RUNNING", 2: "SUCCESS", 3: "FAILURE", 4: "SKIPPED"}.get(value, "?")

    def _on_prompt(self, message: UserPrompt) -> None:
        with self._lock:
            # An empty id means the previous prompt was withdrawn.
            self._prompt = (
                {}
                if not message.prompt_id
                else {
                    "id": message.prompt_id,
                    "message": message.message,
                    "choices": list(message.choices),
                    "has_trajectory": message.has_trajectory,
                }
            )

    # ---- helpers used by the HTTP handlers ------------------------------------------------------

    def call(self, name: str, request, timeout: float = 5.0):
        client = self._service_clients[name]
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        # The executor is spinning on its own thread, so waiting on the future here is safe -- this
        # is the HTTP thread, not a ROS callback.
        if not self._wait(future, timeout):
            return None
        return future.result()

    @staticmethod
    def _wait(future, timeout: float) -> bool:
        import time

        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return False

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "objective": dict(self._state),
                "tree": dict(self._tree),
                "statuses": {str(k): v for k, v in self._statuses.items()},
                "prompt": dict(self._prompt),
                "log": list(self._log[-40:]),
            }

    def run_objective(self, name: str, parameters: dict[str, str]) -> str:
        if not self._action.wait_for_server(timeout_sec=3.0):
            return "no objective server"
        goal = ExecuteObjective.Goal()
        goal.objective_name = name
        goal.parameters = [
            Parameter(name=key, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value)))
            for key, value in parameters.items()
        ]
        future = self._action.send_goal_async(goal)
        if not self._wait(future, 10.0):
            return "the server did not accept the goal"
        handle = future.result()
        if handle is None or not handle.accepted:
            return "goal rejected -- is another Objective already running?"
        self._goal_handle = handle
        return ""

    def cancel_objective(self) -> str:
        if self._goal_handle is None:
            return "nothing running"
        self._goal_handle.cancel_goal_async()
        return ""

    def answer_prompt(self, prompt_id: str, choice: str) -> None:
        message = UserResponse()
        message.prompt_id = prompt_id
        message.choice = choice
        self._response_pub.publish(message)


class StudioServer(ThreadingHTTPServer):
    # Without this, restarting the Studio inside a minute fails on a socket still in TIME_WAIT --
    # which during development is most restarts.
    allow_reuse_address = True


class StudioHandler(BaseHTTPRequestHandler):
    node: StudioNode
    web_root: str

    def log_message(self, *_args):  # noqa: D102 - silence per-request logging
        pass

    # ---- plumbing ------------------------------------------------------------------------------

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload: Any, code: int = 200) -> None:
        self._send(code, json.dumps(payload).encode(), "application/json")

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", 0))
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length))
        except json.JSONDecodeError:
            return {}

    # ---- routes --------------------------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 - required name
        path = self.path.split("?")[0]

        if path == "/api/state":
            self._json(self._state_payload())
        elif path == "/api/behaviors":
            self._json(self._behaviors_payload())
        elif path.startswith("/api/objective/"):
            self._json(self._objective_payload(path.rsplit("/", 1)[-1]))
        elif path.startswith("/api/"):
            self._json({"error": "not found"}, 404)
        else:
            self._static(path)

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?")[0]
        body = self._body()

        if path == "/api/run":
            error = self.node.run_objective(body.get("name", ""), body.get("parameters", {}))
            self._json({"ok": not error, "error": error})
        elif path == "/api/cancel":
            error = self.node.cancel_objective()
            self._json({"ok": not error, "error": error})
        elif path == "/api/answer":
            self.node.answer_prompt(body.get("id", ""), body.get("choice", ""))
            self._json({"ok": True})
        elif path == "/api/validate":
            self._json(self._validate(body.get("xml", "")))
        elif path.startswith("/api/objective/"):
            self._json(self._save(path.rsplit("/", 1)[-1], body))
        elif path == "/api/tool":
            response = self.node.call("switch_tool", SwitchTool.Request(name=body.get("name", "")))
            self._json({"ok": bool(response and response.success),
                        "error": response.message if response else "tool manager did not answer"})
        elif path == "/api/waypoint":
            self._json(self._waypoint(body))
        else:
            self._json({"error": "not found"}, 404)

    # ---- payloads ------------------------------------------------------------------------------

    #: Only these are served. An allowlist rather than mimetypes.guess_type because this process
    #: sits on the robot's network and has no business serving whatever happens to be in the
    #: directory.
    CONTENT_TYPES = {
        ".html": "text/html; charset=utf-8",
        ".js": "application/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8",
        ".svg": "image/svg+xml",
        ".png": "image/png",
        ".woff2": "font/woff2",
        ".json": "application/json",
        ".map": "application/json",
    }

    def _static(self, url_path: str) -> None:
        """Serve a built asset out of web_root.

        The bundler emits content-hashed filenames, so the old fixed three-file routing could not
        serve them. Resolution goes through realpath and is checked against web_root: a request for
        /../../etc/passwd must not escape, and string prefix checks on unresolved paths are exactly
        how that goes wrong.
        """
        relative = url_path.lstrip("/") or "index.html"
        root = os.path.realpath(self.web_root)
        target = os.path.realpath(os.path.join(root, relative))
        if target != root and not target.startswith(root + os.sep):
            self._json({"error": "not found"}, 404)
            return

        content_type = self.CONTENT_TYPES.get(os.path.splitext(target)[1].lower())
        if content_type is None:
            self._json({"error": "not found"}, 404)
            return

        try:
            with open(target, "rb") as handle:
                self._send(200, handle.read(), content_type)
        except OSError:
            self._json({"error": f"{relative} is missing from {self.web_root}"}, 404)

    def _state_payload(self) -> dict[str, Any]:
        payload = self.node.snapshot()

        objectives = self.node.call("list_objectives", ListObjectives.Request())
        payload["objectives"] = (
            [
                {
                    "name": o.name,
                    "description": o.description,
                    "required": list(o.required_parameters),
                    "missing": list(o.missing_behaviors),
                }
                for o in objectives.objectives
            ]
            if objectives
            else []
        )

        tools = self.node.call("list_tools", ListTools.Request())
        payload["tools"] = (
            [{"name": t.name, "description": t.description, "attached": t.attached} for t in tools.tools]
            if tools
            else []
        )

        waypoints = self.node.call("list_waypoints", ListWaypoints.Request())
        payload["waypoints"] = (
            [{"name": w.name, "tags": list(w.tags), "has_pose": w.has_pose} for w in waypoints.waypoints]
            if waypoints
            else []
        )
        return payload

    def _behaviors_payload(self) -> dict[str, Any]:
        response = self.node.call("list_behaviors", ListBehaviors.Request(), timeout=10.0)
        if not response:
            return {"behaviors": []}
        return {
            "behaviors": [
                {
                    "name": b.registration_name,
                    "type": b.node_type,
                    "description": b.description,
                    "package": b.loader_package,
                    "ports": [
                        {"name": p.name, "direction": p.direction, "type": p.type_name,
                         "default": p.default_value, "description": p.description}
                        for p in b.ports
                    ],
                }
                for b in response.behaviors
            ]
        }

    def _objective_payload(self, name: str) -> dict[str, Any]:
        response = self.node.call("get_objective", GetObjective.Request(name=name))
        if not response or not response.found:
            return {"found": False, "xml": ""}
        return {"found": True, "xml": response.xml, "description": response.info.description}

    def _validate(self, xml: str) -> dict[str, Any]:
        response = self.node.call("validate", ValidateObjectiveXml.Request(xml=xml), timeout=10.0)
        if not response:
            return {"valid": False, "xml_error": "the objective server did not answer"}
        return {
            "valid": response.valid,
            "xml_error": response.xml_error,
            "missing_behaviors": list(response.missing_behaviors),
            "unknown_ports": list(response.unknown_ports),
            "invalid_port_values": list(response.invalid_port_values),
        }

    def _save(self, name: str, body: dict[str, Any]) -> dict[str, Any]:
        request = SaveObjectiveXml.Request(name=name, xml=body.get("xml", ""),
                                           file_path=body.get("file_path", ""), reload=True)
        response = self.node.call("save", request, timeout=10.0)
        if not response:
            return {"ok": False, "error": "the objective server did not answer"}
        return {
            "ok": response.success,
            "error": response.message,
            "written_path": response.written_path,
            "backup_path": response.backup_path,
            "xml_error": response.xml_error,
            "missing_behaviors": list(response.missing_behaviors),
            "unknown_ports": list(response.unknown_ports),
            "invalid_port_values": list(response.invalid_port_values),
        }

    def _waypoint(self, body: dict[str, Any]) -> dict[str, Any]:
        action = body.get("action", "teach")
        if action == "delete":
            response = self.node.call("delete_waypoint", DeleteWaypoint.Request(name=body.get("name", "")))
        else:
            request = SaveWaypoint.Request()
            request.name = body.get("name", "")
            request.capture = SaveWaypoint.Request.CAPTURE_CURRENT_BOTH
            request.overwrite = bool(body.get("overwrite", False))
            request.tags = [t for t in body.get("tags", "").split(";") if t]
            request.record_carrier_diagnostics = True
            response = self.node.call("save_waypoint", request)
        if not response:
            return {"ok": False, "error": "the waypoint manager did not answer"}
        return {"ok": response.success, "error": response.message}


def main() -> None:
    rclpy.init()
    node = StudioNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    port = int(os.environ.get("STUDIO_PORT", "8080"))
    web_root = os.path.join(get_package_share_directory("moveit2_extended_studio"), "web")

    StudioHandler.node = node
    StudioHandler.web_root = web_root
    try:
        server = StudioServer(("0.0.0.0", port), StudioHandler)
    except OSError as error:
        # Say which port and what to do about it. The bare "Address already in use" traceback sends
        # people looking at ROS, and the previous instance is still serving a stale page while they
        # look.
        node.get_logger().error(
            f"cannot listen on port {port}: {error}. Another Studio is probably still running "
            f"(pkill -f studio_server), or set STUDIO_PORT to something else."
        )
        rclpy.shutdown()
        return

    node.get_logger().info(f"Studio on http://localhost:{port}  (serving {web_root})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
