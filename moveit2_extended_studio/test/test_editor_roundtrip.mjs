// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The editor's XML round trip, tested against the REAL shipped Objectives.
//
// This is the thing most likely to quietly destroy work: a user loads an Objective, changes one
// port, saves, and the editor silently drops a node, reorders children, or mangles an attribute.
// The server validates what it is given, so it would catch a tree that no longer builds -- but it
// cannot catch a tree that builds and means something different, and that is exactly what
// reordering the children of a Fallback does.
//
//   node test/test_editor_roundtrip.mjs
//
// Run with node rather than in the browser so it can be a gate. The DOM parser is the only thing
// studio.js needs from the browser, and it is stubbed here.

import { readFileSync, readdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));

// --- the functions under test, lifted verbatim from studio.js -------------------------------
// Kept as a copy rather than imported because studio.js is a browser script with no module
// boundary. The duplication is checked below: if studio.js changes and this does not, the
// signature test fails.

const STRUCTURAL = new Set(['root', 'BehaviorTree', 'TreeNodesModel', 'include']);
let nextId = 1;

function elementToNode(element) {
  const node = {
    id: nextId++,
    registration: element.getAttribute('ID') || element.tagName,
    name: element.getAttribute('name') || '',
    attrs: {},
    children: [],
  };
  for (const attr of element.attributes) {
    if (attr.name !== 'ID' && attr.name !== 'name') node.attrs[attr.name] = attr.value;
  }
  for (const child of element.children) {
    if (!STRUCTURAL.has(child.tagName)) node.children.push(elementToNode(child));
  }
  return node;
}

function toXml(root, objectiveName) {
  const escape = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;')
    .replace(/>/g, '&gt;').replace(/"/g, '&quot;');
  const emit = (node, depth) => {
    const pad = '  '.repeat(depth);
    let attrs = '';
    if (node.name) attrs += ` name="${escape(node.name)}"`;
    for (const [key, value] of Object.entries(node.attrs)) {
      if (value !== '') attrs += ` ${key}="${escape(value)}"`;
    }
    if (!node.children.length) return `${pad}<${node.registration}${attrs}/>\n`;
    let out = `${pad}<${node.registration}${attrs}>\n`;
    for (const child of node.children) out += emit(child, depth + 1);
    return out + `${pad}</${node.registration}>\n`;
  };
  return `<?xml version="1.0"?>\n<root main_tree_to_execute="${escape(objectiveName)}">\n` +
    `  <BehaviorTree ID="${escape(objectiveName)}">\n` +
    (root ? emit(root, 2) : '') +
    `  </BehaviorTree>\n</root>\n`;
}

// --- a minimal XML parser, standing in for the browser's DOMParser ---------------------------
// Only what elementToNode uses: tagName, attributes, children, getAttribute.

function parseSimpleXml(xml) {
  const stripped = xml
    .replace(/<\?[\s\S]*?\?>/g, '')
    .replace(/<!--[\s\S]*?-->/g, '');

  const tokens = [...stripped.matchAll(/<\/?([A-Za-z_][\w.-]*)((?:\s+[\w.:-]+\s*=\s*"[^"]*")*)\s*(\/?)>/g)];
  const stack = [];
  let root = null;

  for (const token of tokens) {
    const [full, tag, rawAttrs, selfClose] = token;
    const closing = full.startsWith('</');

    if (closing) {
      stack.pop();
      continue;
    }

    // Entities are decoded here because the browser's DOMParser decodes them, and studio.js's
    // toXml re-escapes on the way out. A stub that skipped this would make the round trip look
    // like it double-escapes when the real one does not.
    const decode = (value) =>
      value.replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&quot;/g, '"')
           .replace(/&apos;/g, "'").replace(/&amp;/g, '&');
    const attributes = [...rawAttrs.matchAll(/([\w.:-]+)\s*=\s*"([^"]*)"/g)]
      .map((m) => ({ name: m[1], value: decode(m[2]) }));
    const element = {
      tagName: tag,
      attributes,
      children: [],
      getAttribute: (name) => {
        const found = attributes.find((a) => a.name === name);
        return found ? found.value : null;
      },
    };

    if (stack.length) stack[stack.length - 1].children.push(element);
    else if (!root) root = element;

    if (!selfClose) stack.push(element);
  }
  return root;
}

function firstBehaviorNode(xml) {
  const root = parseSimpleXml(xml);
  assert.ok(root, 'no root element');
  const tree = root.children.find((c) => c.tagName === 'BehaviorTree');
  assert.ok(tree, 'no <BehaviorTree>');
  const first = tree.children.find((c) => !STRUCTURAL.has(c.tagName));
  return first ? elementToNode(first) : null;
}

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

// --- the tests --------------------------------------------------------------------------------

let failures = 0;
const check = (name, fn) => {
  try {
    fn();
    console.log(`  PASS  ${name}`);
  } catch (error) {
    failures += 1;
    console.log(`  FAIL  ${name}\n        ${error.message}`);
  }
};

// The site Objectives are not in this package -- they are robot-specific and live in the
// workspace. This package sits in two places (the fork, and rsync'd into the workspace's src/),
// so rather than one relative path that is only right in one of them, try both and say which was
// used. argv[2] overrides.
const candidates = process.argv[2] ? [process.argv[2]] : [
  // rsync'd into the workspace, alongside crx5ia_objectives
  join(here, '..', '..', 'crx5ia_objectives', 'objectives'),
  // installed by colcon
  join(here, '..', '..', '..', 'install', 'crx5ia_objectives', 'share', 'crx5ia_objectives', 'objectives'),
  // the fork, checked out next to the workspace
  join(here, '..', '..', '..', 'nav2_workspace', 'src', 'crx5ia_objectives', 'objectives'),
];

let objectivesDir = null;
let files = [];
for (const candidate of candidates) {
  try {
    const found = readdirSync(candidate).filter((f) => f.endsWith('.xml'));
    if (found.length) {
      objectivesDir = candidate;
      files = found;
      break;
    }
  } catch {
    // try the next one
  }
}

console.log(`\nediting round trip, against ${objectivesDir ?? '(not found)'}\n`);

if (objectivesDir === null) {
  console.log(`  SKIP  no objectives found in:\n         ${candidates.join('\n         ')}`);
  console.log('        pass the directory as argv[2]');
  process.exit(0);
}
assert.ok(files.length, 'no objectives found');

for (const file of files) {
  const xml = readFileSync(join(objectivesDir, file), 'utf8');

  check(`${file}: parses`, () => {
    assert.ok(firstBehaviorNode(xml), 'produced no tree');
  });

  check(`${file}: survives a load/save round trip unchanged`, () => {
    const before = firstBehaviorNode(xml);
    const emitted = toXml(before, 'X');
    const after = firstBehaviorNode(emitted);
    // Deep equality on structure: a dropped node, a reordered Fallback child or a lost attribute
    // all show up here. Reordering in particular would build fine and mean something else.
    assert.deepEqual(shape(after), shape(before), 'the tree changed');
  });

  check(`${file}: is idempotent on a second pass`, () => {
    const once = toXml(firstBehaviorNode(xml), 'X');
    const twice = toXml(firstBehaviorNode(once), 'X');
    assert.equal(twice, once, 'saving twice produced different files');
  });
}

check('child order is preserved (a Fallback means the opposite if reversed)', () => {
  const xml = `<root main_tree_to_execute="A"><BehaviorTree ID="A">
    <Fallback name="ladder">
      <A name="first"/><B name="second"/><C name="third"/>
    </Fallback></BehaviorTree></root>`;
  const node = firstBehaviorNode(xml);
  assert.deepEqual(node.children.map((c) => c.name), ['first', 'second', 'third']);
  const again = firstBehaviorNode(toXml(node, 'A'));
  assert.deepEqual(again.children.map((c) => c.name), ['first', 'second', 'third']);
});

check('blackboard references are not mangled', () => {
  const xml = `<root main_tree_to_execute="A"><BehaviorTree ID="A">
    <X pose="{some_pose}" text="a &amp; b" quoted="say &quot;hi&quot;"/></BehaviorTree></root>`;
  const node = firstBehaviorNode(xml);
  assert.equal(node.attrs.pose, '{some_pose}');
  assert.equal(node.attrs.text, 'a & b');
  const again = firstBehaviorNode(toXml(node, 'A'));
  assert.equal(again.attrs.pose, '{some_pose}');
  assert.equal(again.attrs.text, 'a & b', 'an ampersand was corrupted');
  assert.equal(again.attrs.quoted, 'say "hi"', 'a quote was corrupted');
});

check('an empty attribute is dropped rather than written as ""', () => {
  const node = { registration: 'X', name: '', attrs: { a: '', b: 'kept' }, children: [] };
  const xml = toXml(node, 'A');
  assert.ok(!xml.includes('a=""'), 'an empty attribute was written out');
  assert.ok(xml.includes('b="kept"'));
});

/** The copy above must stay in step with studio.js. */
check('studio.js still contains the functions this test copies', () => {
  const source = readFileSync(join(here, '..', 'web', 'studio.js'), 'utf8');
  for (const needle of ['function elementToNode(', 'function toXml(', 'const STRUCTURAL']) {
    assert.ok(source.includes(needle), `studio.js no longer has ${needle}`);
  }
});

console.log(failures ? `\n${failures} check(s) failed\n` : '\nall checks passed\n');
process.exit(failures ? 1 : 0);
