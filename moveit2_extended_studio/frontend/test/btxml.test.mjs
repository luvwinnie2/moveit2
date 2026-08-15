// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The editor's round trip, tested against the REAL shipped Objectives.
//
// This is the thing most likely to quietly destroy work: someone loads an Objective, changes one
// port, saves, and the editor has silently dropped a node, reordered a Fallback's children, or
// mangled an attribute. The server validates what it is given, so it would catch a tree that no
// longer builds -- but it cannot catch a tree that builds and means something different, and
// reordering the children of a Fallback is exactly that. A Fallback that tries its recovery first
// is a valid tree and a broken robot.
//
// Unlike the previous editor's test, this imports the SAME code the browser runs: btxml.ts has its
// own XML reader precisely so there is no DOM to stub and no stub to drift.

import { readFileSync, readdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";
import assert from "node:assert/strict";

import {
  cloneTree,
  findNode,
  findParent,
  flatten,
  insertChild,
  isAncestor,
  moveSibling,
  parseObjective,
  removeNode,
  reparent,
  serializeObjective,
  blackboardKey,
} from "./build/btxml.js";

const here = dirname(fileURLToPath(import.meta.url));

/** Structure only: registration, name, ordered children, and the attribute set. */
function shape(node) {
  if (!node) return null;
  return {
    registration: node.registration,
    name: node.name,
    attrs: Object.fromEntries(Object.entries(node.attrs).sort()),
    children: node.children.map(shape),
  };
}

// The site Objectives live in the workspace, not in this package -- they are robot-specific.
const candidates = [
  join(here, "..", "..", "..", "..", "nav2_workspace", "src", "crx5ia_objectives", "objectives"),
  "/misc/Work39_SSD/cheesiang_leow/nav2_workspace/src/crx5ia_objectives/objectives",
];
let objectivesDir = null;
let files = [];
for (const candidate of candidates) {
  try {
    const found = readdirSync(candidate).filter((f) => f.endsWith(".xml"));
    if (found.length) {
      objectivesDir = candidate;
      files = found;
      break;
    }
  } catch {
    /* try the next */
  }
}

test("the real Objectives are reachable", () => {
  assert.ok(objectivesDir, `no objectives found in:\n  ${candidates.join("\n  ")}`);
  assert.ok(files.length >= 3, `expected several Objectives, found ${files.length}`);
});

for (const file of files) {
  const xml = objectivesDir ? readFileSync(join(objectivesDir, file), "utf8") : "";

  test(`${file}: parses`, () => {
    const { root } = parseObjective(xml);
    assert.ok(root, "produced no tree");
    assert.ok(flatten(root).length > 1, "produced a tree with no children");
  });

  test(`${file}: survives a load/save round trip unchanged`, () => {
    const before = parseObjective(xml);
    const after = parseObjective(serializeObjective(before.root, before.treeId));
    assert.deepEqual(shape(after.root), shape(before.root), "the tree changed");
    assert.equal(after.treeId, before.treeId, "the tree ID changed");
  });

  test(`${file}: is idempotent on a second pass`, () => {
    const first = parseObjective(xml);
    const once = serializeObjective(first.root, first.treeId);
    const second = parseObjective(once);
    assert.equal(serializeObjective(second.root, second.treeId), once, "saving twice differed");
  });
}

// --- the semantics that a generic graph editor gets wrong ---------------------------------------

const ORDERED = `<root main_tree_to_execute="A"><BehaviorTree ID="A">
  <Fallback name="ladder">
    <Sequence name="nominal"/><Sequence name="recovery"/><Sequence name="abort"/>
  </Fallback></BehaviorTree></root>`;

test("child order is preserved -- a Fallback means the opposite if reversed", () => {
  const { root, treeId } = parseObjective(ORDERED);
  assert.deepEqual(root.children.map((c) => c.name), ["nominal", "recovery", "abort"]);
  const again = parseObjective(serializeObjective(root, treeId)).root;
  assert.deepEqual(again.children.map((c) => c.name), ["nominal", "recovery", "abort"]);
});

test("moveSibling changes the order and nothing else", () => {
  const { root } = parseObjective(ORDERED);
  const recovery = root.children[1];
  const moved = moveSibling(root, recovery.id, -1);
  assert.deepEqual(moved.children.map((c) => c.name), ["recovery", "nominal", "abort"]);
  assert.equal(flatten(moved).length, flatten(root).length, "a node appeared or vanished");
  assert.deepEqual(root.children.map((c) => c.name), ["nominal", "recovery", "abort"], "the input was mutated");
});

test("moveSibling refuses to run off either end", () => {
  const { root } = parseObjective(ORDERED);
  assert.equal(moveSibling(root, root.children[0].id, -1), root, "moved the first child earlier");
  assert.equal(moveSibling(root, root.children[2].id, 1), root, "moved the last child later");
});

test("reparent refuses to put a node inside its own subtree", () => {
  const { root } = parseObjective(ORDERED);
  const child = root.children[0];
  assert.ok(isAncestor(root, root.id, child.id));
  assert.equal(reparent(root, root.id, child.id), root, "made a cycle");
  assert.equal(reparent(root, child.id, child.id), root, "re-parented onto itself");
});

test("reparent moves the whole subtree, once", () => {
  const { root } = parseObjective(ORDERED);
  const withChild = insertChild(root, root.children[0].id, {
    id: "leaf",
    registration: "LogMessage",
    name: "note",
    attrs: {},
    children: [],
  });
  const nominal = withChild.children[0];
  const moved = reparent(withChild, nominal.id, withChild.children[2].id);
  assert.equal(flatten(moved).filter((n) => n.name === "note").length, 1, "the subtree was duplicated");
  assert.equal(findParent(moved, findNode(moved, nominal.id).id).name, "abort");
  assert.equal(moved.children.length, 2, "the node was not detached from its old parent");
});

test("removeNode takes the subtree with it", () => {
  const { root } = parseObjective(ORDERED);
  const smaller = removeNode(root, root.children[1].id);
  assert.deepEqual(smaller.children.map((c) => c.name), ["nominal", "abort"]);
});

// --- text handling ------------------------------------------------------------------------------

test("blackboard references and entities are not mangled", () => {
  const xml = `<root main_tree_to_execute="A"><BehaviorTree ID="A">
    <X pose="{some_pose}" text="a &amp; b" quoted="say &quot;hi&quot;" lt="1 &lt; 2"/></BehaviorTree></root>`;
  const { root, treeId } = parseObjective(xml);
  assert.equal(root.attrs.pose, "{some_pose}");
  assert.equal(root.attrs.text, "a & b");
  assert.equal(root.attrs.quoted, 'say "hi"');
  assert.equal(root.attrs.lt, "1 < 2");

  const again = parseObjective(serializeObjective(root, treeId)).root;
  assert.equal(again.attrs.pose, "{some_pose}", "a blackboard reference was corrupted");
  assert.equal(again.attrs.text, "a & b", "an ampersand was corrupted");
  assert.equal(again.attrs.quoted, 'say "hi"', "a quote was corrupted");
  assert.equal(again.attrs.lt, "1 < 2", "a less-than was corrupted");
});

test("an ampersand entity is not double-decoded", () => {
  // "&amp;lt;" is the text "&lt;", not the character "<".
  const xml = `<root main_tree_to_execute="A"><BehaviorTree ID="A"><X v="&amp;lt;"/></BehaviorTree></root>`;
  assert.equal(parseObjective(xml).root.attrs.v, "&lt;");
});

test("an empty attribute is dropped rather than written as an empty string", () => {
  const node = { id: "x", registration: "X", name: "", attrs: { a: "", b: "kept" }, children: [] };
  const xml = serializeObjective(node, "A");
  assert.ok(!xml.includes('a=""'), "an empty attribute was written out");
  assert.ok(xml.includes('b="kept"'));
});

test("blackboardKey recognises a reference and only a reference", () => {
  assert.equal(blackboardKey("{waypoint}"), "waypoint");
  assert.equal(blackboardKey("  {waypoint}  "), "waypoint");
  assert.equal(blackboardKey("waypoint"), null);
  assert.equal(blackboardKey("{a b}"), null);
  assert.equal(blackboardKey("prefix {a}"), null);
});

test("cloneTree is deep -- editing the copy must not touch the original", () => {
  const { root } = parseObjective(ORDERED);
  const copy = cloneTree(root);
  copy.children[0].name = "changed";
  copy.attrs.added = "1";
  assert.equal(root.children[0].name, "nominal");
  assert.equal(root.attrs.added, undefined);
});

test("comments outside <root> do not become nodes", () => {
  const xml = `<?xml version="1.0"?>\n<!-- why this tree exists -->\n${ORDERED}`;
  const { root } = parseObjective(xml);
  assert.equal(root.registration, "Fallback");
  assert.equal(root.children.length, 3);
});
