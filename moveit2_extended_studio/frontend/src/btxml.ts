// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// BehaviorTree XML <-> editor model.
//
// The parser here is hand-written rather than DOMParser on purpose. The previous editor used the
// browser's DOMParser and its round-trip test had to fake a DOM in Node, so the thing under test
// was never quite the thing that shipped -- and the stub got entity decoding wrong at least once.
// One parser, identical in the browser and in `node --test`, removes that whole class of bug.
//
// The one rule that matters throughout: CHILD ORDER IS MEANING. A Fallback lists its recovery
// after its nominal path; swap them and the tree tries to recover from a failure that has not
// happened yet. Nothing in this file may reorder children, and the tests assert it.

import type { BtNode } from "./types";

/** Tags that are document structure, not Behaviors. */
const STRUCTURAL = new Set(["root", "BehaviorTree", "TreeNodesModel", "include"]);

/** Attributes that name the node rather than feeding a port. */
const RESERVED = new Set(["ID", "name"]);

let nextId = 1;
export function freshId(): string {
  return `n${nextId++}`;
}

// --- a minimal XML reader -----------------------------------------------------------------------

interface RawElement {
  tag: string;
  attrs: [string, string][];
  children: RawElement[];
}

function decodeEntities(value: string): string {
  return value
    .replace(/&lt;/g, "<")
    .replace(/&gt;/g, ">")
    .replace(/&quot;/g, '"')
    .replace(/&apos;/g, "'")
    .replace(/&#(\d+);/g, (_, code: string) => String.fromCodePoint(Number(code)))
    // Ampersand last, so "&amp;lt;" decodes to "&lt;" and not to "<".
    .replace(/&amp;/g, "&");
}

export function escapeXml(value: string): string {
  return String(value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

const TAG = /<\/?([A-Za-z_][\w.\-]*)((?:\s+[\w.:\-]+\s*=\s*"[^"]*")*)\s*(\/?)>/g;
const ATTR = /([\w.:\-]+)\s*=\s*"([^"]*)"/g;

/** Parse to a raw element tree. Returns null when there is no root element. */
function readXml(xml: string): RawElement | null {
  const stripped = xml.replace(/<\?[\s\S]*?\?>/g, "").replace(/<!--[\s\S]*?-->/g, "");

  const stack: RawElement[] = [];
  let root: RawElement | null = null;

  TAG.lastIndex = 0;
  for (let match = TAG.exec(stripped); match !== null; match = TAG.exec(stripped)) {
    const [full, tag, rawAttrs, selfClose] = match;
    if (tag === undefined) continue;

    if (full.startsWith("</")) {
      stack.pop();
      continue;
    }

    const attrs: [string, string][] = [];
    ATTR.lastIndex = 0;
    for (let a = ATTR.exec(rawAttrs ?? ""); a !== null; a = ATTR.exec(rawAttrs ?? "")) {
      if (a[1] !== undefined && a[2] !== undefined) attrs.push([a[1], decodeEntities(a[2])]);
    }

    const element: RawElement = { tag, attrs, children: [] };
    const parent = stack[stack.length - 1];
    if (parent) parent.children.push(element);
    else if (!root) root = element;

    if (!selfClose) stack.push(element);
  }
  return root;
}

// --- model <-> XML ------------------------------------------------------------------------------

function toNode(element: RawElement): BtNode {
  const byName = new Map(element.attrs);
  const node: BtNode = {
    id: freshId(),
    registration: byName.get("ID") || element.tag,
    name: byName.get("name") ?? "",
    attrs: {},
    children: [],
  };
  for (const [key, value] of element.attrs) {
    if (!RESERVED.has(key)) node.attrs[key] = value;
  }
  // In document order, which is child order, which is execution order.
  for (const child of element.children) {
    if (!STRUCTURAL.has(child.tag)) node.children.push(toNode(child));
  }
  return node;
}

/** The tree inside the first <BehaviorTree>, or null if the document has none. */
export function parseObjective(xml: string): { root: BtNode | null; treeId: string } {
  const document = readXml(xml);
  if (!document) return { root: null, treeId: "" };

  const trees =
    document.tag === "BehaviorTree"
      ? [document]
      : document.children.filter((c) => c.tag === "BehaviorTree");
  const tree = trees[0];
  if (!tree) return { root: null, treeId: "" };

  const treeId = new Map(tree.attrs).get("ID") ?? "";
  const first = tree.children.find((c) => !STRUCTURAL.has(c.tag));
  return { root: first ? toNode(first) : null, treeId };
}

export function serializeObjective(root: BtNode | null, treeId: string, header = ""): string {
  const emit = (node: BtNode, depth: number): string => {
    const pad = "  ".repeat(depth);
    let attrs = "";
    if (node.name) attrs += ` name="${escapeXml(node.name)}"`;
    for (const [key, value] of Object.entries(node.attrs)) {
      // An empty value is not the same as an unset port: writing key="" makes BehaviorTree.CPP
      // try to parse "" as the port's type and fail at run time.
      if (value !== "") attrs += ` ${key}="${escapeXml(value)}"`;
    }
    if (node.children.length === 0) return `${pad}<${node.registration}${attrs}/>\n`;
    let out = `${pad}<${node.registration}${attrs}>\n`;
    for (const child of node.children) out += emit(child, depth + 1);
    return `${out}${pad}</${node.registration}>\n`;
  };

  const id = escapeXml(treeId || "Objective");
  return (
    `<?xml version="1.0"?>\n` +
    (header ? `${header}\n` : "") +
    `<root main_tree_to_execute="${id}">\n` +
    `  <BehaviorTree ID="${id}">\n` +
    (root ? emit(root, 2) : "") +
    `  </BehaviorTree>\n</root>\n`
  );
}

/** Comments before <root>, so editing does not throw away the file's explanation of itself. */
export function extractHeader(xml: string): string {
  const comments: string[] = [];
  const upToRoot = xml.split(/<root\b/)[0] ?? "";
  const pattern = /<!--[\s\S]*?-->/g;
  for (let m = pattern.exec(upToRoot); m !== null; m = pattern.exec(upToRoot)) comments.push(m[0]);
  return comments.join("\n");
}

// --- tree operations ----------------------------------------------------------------------------
//
// All of these return a NEW tree. React state wants immutability, and an in-place edit of a node
// deep in the tree is exactly the kind of change React will not re-render.

export function cloneTree(node: BtNode): BtNode {
  return { ...node, attrs: { ...node.attrs }, children: node.children.map(cloneTree) };
}

export function findNode(root: BtNode | null, id: string): BtNode | null {
  if (!root) return null;
  if (root.id === id) return root;
  for (const child of root.children) {
    const found = findNode(child, id);
    if (found) return found;
  }
  return null;
}

export function findParent(root: BtNode | null, id: string): BtNode | null {
  if (!root) return null;
  for (const child of root.children) {
    if (child.id === id) return root;
    const found = findParent(child, id);
    if (found) return found;
  }
  return null;
}

/** Depth-first list, parents before children -- the order the runtime reports too. */
export function flatten(root: BtNode | null): BtNode[] {
  if (!root) return [];
  return [root, ...root.children.flatMap(flatten)];
}

export function updateNode(root: BtNode, id: string, change: (node: BtNode) => void): BtNode {
  const copy = cloneTree(root);
  const target = findNode(copy, id);
  if (target) change(target);
  return copy;
}

export function insertChild(root: BtNode, parentId: string, child: BtNode, index?: number): BtNode {
  const copy = cloneTree(root);
  const parent = findNode(copy, parentId);
  if (!parent) return root;
  const at = index === undefined ? parent.children.length : Math.max(0, Math.min(index, parent.children.length));
  parent.children.splice(at, 0, child);
  return copy;
}

export function removeNode(root: BtNode, id: string): BtNode | null {
  if (root.id === id) return null; // the caller decides what an empty document means
  const copy = cloneTree(root);
  const parent = findParent(copy, id);
  if (!parent) return root;
  parent.children = parent.children.filter((c) => c.id !== id);
  return copy;
}

/** Move a child one place earlier or later among its siblings. This is a semantic edit. */
export function moveSibling(root: BtNode, id: string, delta: number): BtNode {
  const copy = cloneTree(root);
  const parent = findParent(copy, id);
  if (!parent) return root;
  const from = parent.children.findIndex((c) => c.id === id);
  const to = from + delta;
  if (from < 0 || to < 0 || to >= parent.children.length) return root;
  const [moved] = parent.children.splice(from, 1);
  if (moved) parent.children.splice(to, 0, moved);
  return copy;
}

/** True when `ancestorId` is at or above `id`; re-parenting into your own subtree makes a cycle. */
export function isAncestor(root: BtNode | null, ancestorId: string, id: string): boolean {
  const ancestor = findNode(root, ancestorId);
  return ancestor ? flatten(ancestor).some((n) => n.id === id) : false;
}

export function reparent(root: BtNode, id: string, newParentId: string, index?: number): BtNode {
  if (id === newParentId || isAncestor(root, id, newParentId)) return root;
  const moving = findNode(root, id);
  if (!moving) return root;
  const detached = removeNode(root, id);
  if (!detached) return root;
  return insertChild(detached, newParentId, cloneTree(moving), index);
}

/** Blackboard references are "{key}" exactly; anything else is a literal. */
export function blackboardKey(value: string): string | null {
  const match = /^\{([^{}\s]+)\}$/.exec(value.trim());
  return match && match[1] !== undefined ? match[1] : null;
}
