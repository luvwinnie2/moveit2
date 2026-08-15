// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Tidy-tree layout, deterministic and driven by child index.
//
// React Flow will happily let nodes be dragged anywhere, and a graph editor usually lets them stay
// there. Not here: in a Behavior Tree the left-to-right order of siblings IS the execution order,
// so a node's position is not decoration, it is a reading of the semantics. Letting a user drag a
// recovery branch to the left of the nominal one would draw a tree that lies about what runs first.
// So positions are computed, every time, from the tree.

import type { BtNode } from "./types";

export const NODE_WIDTH = 216;
export const NODE_HEIGHT = 62;
const GAP_X = 28;
const GAP_Y = 46;

export interface Placed {
  node: BtNode;
  /** Descendants hidden because this node is collapsed. 0 when it is not. */
  hidden: number;
  x: number;
  y: number;
  depth: number;
  /** Index among its siblings, i.e. the order it runs in. */
  order: number;
  parentId: string | null;
  siblingCount: number;
}

/**
 * Positions for every node, laid out so that:
 *   - depth becomes y, so the tree reads top to bottom;
 *   - siblings are placed left to right in array order, never sorted;
 *   - a parent sits centred over the block its children occupy.
 *
 * Leaves are packed by a running cursor, which is the simplest layout that never overlaps and
 * stays stable as the tree is edited -- a layout that jumps around on every keystroke is worse
 * than a plain one.
 */
export function layoutTree(root: BtNode | null, collapsed: ReadonlySet<string> = new Set()): Placed[] {
  if (!root) return [];

  const placed: Placed[] = [];
  let cursor = 0;

  const countDescendants = (node: BtNode): number =>
    node.children.length + node.children.reduce((sum, child) => sum + countDescendants(child), 0);

  const walk = (node: BtNode, depth: number, order: number, parentId: string | null, siblingCount: number): number => {
    const isCollapsed = collapsed.has(node.id) && node.children.length > 0;
    const entry: Placed = {
      node,
      hidden: isCollapsed ? countDescendants(node) : 0,
      x: 0,
      y: depth * (NODE_HEIGHT + GAP_Y),
      depth,
      order,
      parentId,
      siblingCount,
    };
    placed.push(entry);

    if (isCollapsed || node.children.length === 0) {
      entry.x = cursor * (NODE_WIDTH + GAP_X);
      cursor += 1;
      return entry.x;
    }

    const childCentres = node.children.map((child, index) =>
      walk(child, depth + 1, index, node.id, node.children.length),
    );
    const first = childCentres[0] ?? 0;
    const last = childCentres[childCentres.length - 1] ?? first;
    entry.x = (first + last) / 2;
    return entry.x;
  };

  walk(root, 0, 0, null, 1);
  return placed;
}

/** Same layout, for the tree the server reports while an Objective runs. */
export interface RuntimePlaced {
  uid: number;
  label: string;
  registration: string;
  type: number;
  x: number;
  y: number;
  order: number;
  parent: number | null;
}

export function layoutRuntime(
  nodes: { uid: number; parent: number; name: string; registration: string; type: number; children: number[] }[],
): RuntimePlaced[] {
  if (nodes.length === 0) return [];
  const byUid = new Map(nodes.map((n) => [n.uid, n]));
  const first = nodes[0];
  if (!first) return [];

  // The server sends depth-first with parents first, so the first entry is the root. A node whose
  // parent is itself (or missing) is also treated as a root, which is how the root marks itself.
  const rootUid = first.uid;
  const placed: RuntimePlaced[] = [];
  let cursor = 0;

  const walk = (uid: number, depth: number, order: number, parent: number | null, seen: Set<number>): number => {
    if (seen.has(uid)) return 0;
    seen.add(uid);
    const node = byUid.get(uid);
    if (!node) return 0;

    const entry: RuntimePlaced = {
      uid,
      label: node.name || node.registration,
      registration: node.registration,
      type: node.type,
      x: 0,
      y: depth * (NODE_HEIGHT + GAP_Y),
      order,
      parent,
    };
    placed.push(entry);

    const children = node.children.filter((c) => c !== uid);
    if (children.length === 0) {
      entry.x = cursor * (NODE_WIDTH + GAP_X);
      cursor += 1;
      return entry.x;
    }
    const centres = children.map((child, index) => walk(child, depth + 1, index, uid, seen));
    const head = centres[0] ?? 0;
    const tail = centres[centres.length - 1] ?? head;
    entry.x = (head + tail) / 2;
    return entry.x;
  };

  walk(rootUid, 0, 0, null, new Set());
  return placed;
}
