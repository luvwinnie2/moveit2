// Copyright 2026 Leow Chee Siang. Apache-2.0.

/** BehaviorTree.CPP node categories, matching TreeNodeInfo's enum. */
export type BtCategory = "action" | "condition" | "control" | "decorator" | "subtree" | "unknown";

/** Numeric node_type from moveit2_extended_msgs/TreeNodeInfo. */
export const CATEGORY_BY_CODE: Record<number, BtCategory> = {
  0: "action",
  1: "condition",
  2: "control",
  3: "decorator",
  4: "subtree",
  5: "unknown",
};

/** Status codes from moveit2_extended_msgs/BehaviorStatus. */
export type BtStatus = "idle" | "running" | "success" | "failure" | "skipped";
export const STATUS_BY_CODE: Record<number, BtStatus> = {
  0: "idle",
  1: "running",
  2: "success",
  3: "failure",
  4: "skipped",
};

/**
 * One node of the tree being edited.
 *
 * `children` is an ORDERED array and that is the whole point of this type. A Fallback whose
 * children are reordered means something completely different -- it tries the recovery before the
 * nominal path -- and a tree built out of an unordered edge set cannot represent that. So the tree
 * here is the single source of truth, and the React Flow graph is derived from it on every render
 * rather than being edited directly.
 */
export interface BtNode {
  /** Editor-local, stable for the lifetime of one document. Not the runtime uid. */
  id: string;
  /** The registered Behavior name, i.e. the XML tag or its ID attribute. */
  registration: string;
  /** The XML name= attribute. Empty means "unnamed"; the registration is shown instead. */
  name: string;
  /** Ports, as written in the XML. A value of "{key}" is a blackboard reference. */
  attrs: Record<string, string>;
  children: BtNode[];
}

/** A port as the objective server describes it. */
export interface PortInfo {
  name: string;
  direction: string; // "input" | "output" | "inout"
  type: string;
  default: string;
  description: string;
}

/** A Behavior the server has registered, i.e. one palette entry. */
export interface BehaviorInfo {
  name: string;
  type: number;
  description: string;
  package: string;
  ports: PortInfo[];
}

export interface ObjectiveInfo {
  name: string;
  description: string;
  required: string[];
  missing: string[];
}

export interface RuntimeNode {
  uid: number;
  parent: number;
  name: string;
  registration: string;
  type: number;
  children: number[];
}

export interface RuntimeTree {
  objective: string;
  instance: string;
  xml: string;
  nodes: RuntimeNode[];
}

export interface ObjectiveState {
  state: string;
  objective: string;
  current_behavior: string;
  ticks: number;
  instance: string;
}

export interface UserPrompt {
  id?: string;
  message?: string;
  choices?: string[];
  has_trajectory?: boolean;
}

export interface StudioState {
  objective: ObjectiveState;
  tree: Partial<RuntimeTree>;
  statuses: Record<string, number>;
  prompt: UserPrompt;
  log: string[];
  objectives: ObjectiveInfo[];
  tools: { name: string; description: string; attached: boolean }[];
  waypoints: { name: string; tags: string[]; has_pose: boolean }[];
}

/** The four separate answers the server gives about a tree, kept separate on purpose. */
export interface Validation {
  valid: boolean;
  xml_error: string;
  missing_behaviors: string[];
  unknown_ports: string[];
  invalid_port_values: string[];
}

export interface SaveResult extends Validation {
  ok: boolean;
  error: string;
  written_path: string;
  backup_path: string;
}
