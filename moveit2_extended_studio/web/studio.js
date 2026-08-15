// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// The Studio front end. No framework, no build step, no CDN: this has to run on a robot with no
// internet, and a build step is one more thing to be broken at the wrong moment.
//
// THE CENTRAL DESIGN DECISION: one tree renderer, two jobs.
//
// While an Objective runs, the tree is drawn from the server's TreeStructure and painted with live
// node status. While nothing runs, the SAME renderer draws a tree parsed from XML and lets you
// edit it. Sharing the layout means the thing you edit appears in exactly the positions you
// watched it run in -- which is most of what makes a tree editor worth having over a text editor.
//
// Why not a graph library: litegraph, rete and friends model dataflow DAGs, where edges carry data
// and order does not matter. A BehaviorTree is a tree whose children are ORDERED, and that order
// is the semantics. Shoehorning it into a DAG editor loses exactly the property that matters.

'use strict';

const state = {
  editing: false,
  behaviors: [],
  // The editable model: { id, registration, name, attrs:{}, children:[] }
  root: null,
  selected: null,
  nextId: 1,
  objectiveName: '',
  liveStatuses: {},
  liveTree: null,
};

const $ = (id) => document.getElementById(id);

async function api(path, options) {
  const response = await fetch(path, options);
  return response.json();
}

// --------------------------------------------------------------------------------------------
// XML <-> model
// --------------------------------------------------------------------------------------------

// Elements that are file structure rather than nodes.
const STRUCTURAL = new Set(['root', 'BehaviorTree', 'TreeNodesModel', 'include']);

function parseXml(xml) {
  const doc = new DOMParser().parseFromString(xml, 'application/xml');
  if (doc.querySelector('parsererror')) return null;
  const tree = doc.querySelector('root > BehaviorTree');
  if (!tree) return null;
  const first = Array.from(tree.children).find((c) => !STRUCTURAL.has(c.tagName));
  return first ? elementToNode(first) : null;
}

function elementToNode(element) {
  const node = {
    id: state.nextId++,
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

// --------------------------------------------------------------------------------------------
// layout and rendering
// --------------------------------------------------------------------------------------------

const NODE_W = 168, NODE_H = 40, GAP_X = 18, GAP_Y = 34;

/** Tidy-tree layout: lay children out left to right, then centre the parent over them. Simple
 *  because a BehaviorTree is small and always a tree -- no edge routing to worry about. */
function layout(node, depth, cursor) {
  node.y = depth * (NODE_H + GAP_Y) + 16;
  if (!node.children.length) {
    node.x = cursor.x;
    cursor.x += NODE_W + GAP_X;
    return;
  }
  for (const child of node.children) layout(child, depth + 1, cursor);
  const first = node.children[0], last = node.children[node.children.length - 1];
  node.x = (first.x + last.x) / 2;
}

function statusClass(uid) {
  switch (state.liveStatuses[uid]) {
    case 1: return ' running';
    case 2: return ' success';
    case 3: return ' failure';
    default: return '';
  }
}

function render() {
  const svg = $('canvas');
  svg.innerHTML = '';
  if (!state.root) {
    svg.setAttribute('width', 400);
    svg.setAttribute('height', 80);
    svg.innerHTML = '<text x="16" y="40" fill="#8d94a3">nothing loaded</text>';
    return;
  }

  const cursor = { x: 16 };
  layout(state.root, 0, cursor);

  let maxX = 0, maxY = 0;
  const walk = (node, fn) => { fn(node); node.children.forEach((c) => walk(c, fn)); };
  walk(state.root, (n) => { maxX = Math.max(maxX, n.x + NODE_W); maxY = Math.max(maxY, n.y + NODE_H); });
  svg.setAttribute('width', maxX + 24);
  svg.setAttribute('height', maxY + 24);

  const ns = 'http://www.w3.org/2000/svg';
  const add = (tag, attrs, parent) => {
    const element = document.createElementNS(ns, tag);
    for (const [k, v] of Object.entries(attrs)) element.setAttribute(k, v);
    (parent || svg).appendChild(element);
    return element;
  };

  walk(state.root, (node) => {
    for (const child of node.children) {
      const x1 = node.x + NODE_W / 2, y1 = node.y + NODE_H;
      const x2 = child.x + NODE_W / 2, y2 = child.y;
      const mid = (y1 + y2) / 2;
      add('path', { class: 'link', d: `M${x1},${y1} L${x1},${mid} L${x2},${mid} L${x2},${y2}` });
    }
  });

  walk(state.root, (node) => {
    const selected = state.selected && state.selected.id === node.id ? ' sel' : '';
    const group = add('g', { class: 'node' + statusClass(node.uid) + selected });
    const rect = add('rect', { x: node.x, y: node.y, width: NODE_W, height: NODE_H, rx: 5 }, group);
    add('text', { x: node.x + 8, y: node.y + 17 }, group).textContent =
      node.name || node.registration;
    const registration = add('text', { x: node.x + 8, y: node.y + 31, class: 'reg' }, group);
    registration.textContent = node.registration + (node.children.length ? ` (${node.children.length})` : '');

    rect.addEventListener('click', () => { select(node); });
    group.addEventListener('dragover', (event) => { if (state.editing) event.preventDefault(); });
    group.addEventListener('drop', (event) => {
      if (!state.editing) return;
      event.preventDefault();
      const registrationName = event.dataTransfer.getData('text/plain');
      if (registrationName) {
        node.children.push({ id: state.nextId++, registration: registrationName, name: '',
                             attrs: {}, children: [] });
        render();
      }
    });
  });
}

function select(node) {
  state.selected = node;
  renderInspector();
  render();
}

// --------------------------------------------------------------------------------------------
// inspector
// --------------------------------------------------------------------------------------------

function renderInspector() {
  const title = $('inspector-title'), ports = $('inspector-ports');
  const node = state.selected;
  if (!node) {
    title.textContent = 'nothing selected';
    ports.innerHTML = '';
    return;
  }
  title.textContent = node.registration;

  const behavior = state.behaviors.find((b) => b.name === node.registration);
  ports.innerHTML = '';

  const nameRow = document.createElement('div');
  nameRow.className = 'port-row';
  nameRow.innerHTML = '<label>name <span class="porttype">instance label</span></label>';
  const nameInput = document.createElement('input');
  nameInput.value = node.name;
  nameInput.disabled = !state.editing;
  nameInput.addEventListener('input', () => { node.name = nameInput.value; render(); });
  nameRow.appendChild(nameInput);
  ports.appendChild(nameRow);

  const declared = behavior ? behavior.ports : [];
  if (!behavior) {
    const warning = document.createElement('div');
    warning.className = 'bad';
    warning.textContent = 'this Behavior is not registered on the server';
    ports.appendChild(warning);
  }

  // Declared ports first, then anything set in the XML that the Behavior does not declare -- those
  // are exactly what the server's validation flags, so showing them is how the user finds a typo.
  const shown = new Set();
  for (const port of declared) {
    shown.add(port.name);
    ports.appendChild(portRow(node, port));
  }
  for (const key of Object.keys(node.attrs)) {
    if (!shown.has(key)) {
      ports.appendChild(portRow(node, { name: key, type: 'not declared by this Behavior',
                                        direction: 0, description: '' }, true));
    }
  }
}

function portRow(node, port, unknown) {
  const row = document.createElement('div');
  row.className = 'port-row';
  const direction = ['in', 'out', 'inout'][port.direction] || '';
  row.innerHTML = `<label>${port.name}` +
    `<span class="porttype${unknown ? ' bad' : ''}"> ${direction} ${port.type || ''}</span></label>`;
  const input = document.createElement('input');
  input.value = node.attrs[port.name] || '';
  input.placeholder = port.default || '';
  input.title = port.description || '';
  input.disabled = !state.editing;
  input.addEventListener('input', () => {
    if (input.value === '') delete node.attrs[port.name];
    else node.attrs[port.name] = input.value;
  });
  row.appendChild(input);
  return row;
}

// --------------------------------------------------------------------------------------------
// palette
// --------------------------------------------------------------------------------------------

function renderPalette(filter) {
  const list = $('palette-list');
  list.innerHTML = '';
  const needle = (filter || '').toLowerCase();
  for (const behavior of state.behaviors) {
    if (needle && !behavior.name.toLowerCase().includes(needle) &&
        !(behavior.package || '').toLowerCase().includes(needle)) continue;
    const item = document.createElement('div');
    item.className = 'palette-item';
    item.draggable = true;
    item.innerHTML = `${behavior.name}<div class="pkg">${behavior.package || ''}</div>`;
    item.title = behavior.description || '';
    item.addEventListener('dragstart', (event) => {
      event.dataTransfer.setData('text/plain', behavior.name);
    });
    list.appendChild(item);
  }
}

// --------------------------------------------------------------------------------------------
// validation
// --------------------------------------------------------------------------------------------

function showValidation(result) {
  const box = $('validation');
  if (result.valid) {
    box.innerHTML = '<span class="ok">valid</span>';
    return;
  }
  let html = '<span class="bad">not valid</span>';
  const section = (title, items) => {
    if (!items || !items.length) return '';
    return `<h4>${title}</h4>` + items.map((i) => `<div class="bad">${i}</div>`).join('');
  };
  if (result.xml_error) html += `<h4>XML</h4><div class="bad">${result.xml_error}</div>`;
  html += section('Behaviors nobody registered', result.missing_behaviors);
  html += section('ports that do not exist', result.unknown_ports);
  // The one BehaviorTree.CPP itself will not catch: it parses literals lazily, so a bad pose only
  // fails when that branch is finally reached.
  html += section('values that will not parse', result.invalid_port_values);
  box.innerHTML = html;
}

// --------------------------------------------------------------------------------------------
// polling
// --------------------------------------------------------------------------------------------

async function poll() {
  try {
    const data = await api('/api/state');

    const status = $('status');
    status.textContent = data.objective.state;
    status.className = 'status ' + data.objective.state.toLowerCase();
    $('current').textContent = data.objective.current_behavior
      ? `${data.objective.objective} -> ${data.objective.current_behavior} (${data.objective.ticks} ticks)`
      : '';

    state.liveStatuses = {};
    for (const [uid, value] of Object.entries(data.statuses || {})) {
      state.liveStatuses[Number(uid)] = value;
    }

    // While something runs, the server's tree is the truth and editing is off.
    if (data.objective.state === 'RUNNING' && data.tree && data.tree.nodes && data.tree.nodes.length) {
      if (!state.editing) {
        state.liveTree = data.tree;
        state.root = liveTreeToModel(data.tree);
        render();
      }
    }

    renderList('objectives', data.objectives);
    renderTools(data.tools);
    renderWaypoints(data.waypoints);
    $('log').innerHTML = (data.log || []).slice().reverse().map((l) => `<div>${l}</div>`).join('');
    renderPrompt(data.prompt);
  } catch (error) {
    $('status').textContent = 'NO SERVER';
    $('status').className = 'status failed';
  }
}

/** The server sends a flat node list with parent links; rebuild the tree so the same renderer can
 *  draw it. uid is carried through, because that is what the status colours key on. */
function liveTreeToModel(tree) {
  const byUid = new Map();
  for (const node of tree.nodes) {
    byUid.set(node.uid, { id: state.nextId++, uid: node.uid, registration: node.registration,
                          name: node.name, attrs: {}, children: [] });
  }
  let root = null;
  for (const node of tree.nodes) {
    const model = byUid.get(node.uid);
    const parent = byUid.get(node.parent);
    if (parent && node.parent !== 0) parent.children.push(model);
    else if (!root) root = model;
  }
  return root;
}

function renderList(id, items) {
  const select = $('objective-list');
  const previous = select.value;
  select.innerHTML = '';
  const editSelect = $('edit-objective');
  editSelect.innerHTML = '';
  for (const item of items || []) {
    const option = document.createElement('option');
    option.value = item.name;
    option.textContent = item.name + (item.missing.length ? '  (not runnable)' : '');
    option.disabled = item.missing.length > 0;
    option.title = item.missing.length ? 'missing: ' + item.missing.join(', ') : item.description;
    select.appendChild(option);

    const editOption = document.createElement('option');
    editOption.value = item.name;
    editOption.textContent = item.name;
    editSelect.appendChild(editOption);
  }
  if (previous) select.value = previous;
  const chosen = (items || []).find((o) => o.name === select.value);
  $('objective-description').textContent = chosen ? chosen.description : '';
}

function renderTools(tools) {
  const box = $('tools');
  box.innerHTML = '';
  for (const tool of tools || []) {
    const chip = document.createElement('div');
    chip.className = 'chip' + (tool.attached ? ' on' : '');
    chip.innerHTML = `<span title="${tool.description || ''}">${tool.name}</span>`;
    const button = document.createElement('button');
    button.textContent = tool.attached ? 'fitted' : 'fit';
    button.disabled = tool.attached;
    button.addEventListener('click', async () => {
      const result = await api('/api/tool', { method: 'POST', body: JSON.stringify({ name: tool.name }) });
      if (!result.ok) alert(result.error);
    });
    chip.appendChild(button);
    box.appendChild(chip);
  }
}

function renderWaypoints(waypoints) {
  const box = $('waypoints');
  box.innerHTML = '';
  for (const waypoint of waypoints || []) {
    const chip = document.createElement('div');
    chip.className = 'chip';
    chip.innerHTML = `<span>${waypoint.name}</span>`;
    const remove = document.createElement('button');
    remove.textContent = 'x';
    remove.addEventListener('click', async () => {
      await api('/api/waypoint', { method: 'POST',
                                   body: JSON.stringify({ action: 'delete', name: waypoint.name }) });
    });
    chip.appendChild(remove);
    box.appendChild(chip);
  }
}

function renderPrompt(prompt) {
  const strip = $('prompt');
  if (!prompt || !prompt.id) {
    strip.classList.add('hidden');
    return;
  }
  strip.classList.remove('hidden');
  $('prompt-message').textContent = prompt.message;
  const buttons = $('prompt-buttons');
  buttons.innerHTML = '';
  for (const choice of prompt.choices) {
    const button = document.createElement('button');
    button.textContent = choice;
    button.addEventListener('click', () => {
      api('/api/answer', { method: 'POST', body: JSON.stringify({ id: prompt.id, choice }) });
      strip.classList.add('hidden');
    });
    buttons.appendChild(button);
  }
}

// --------------------------------------------------------------------------------------------
// wiring
// --------------------------------------------------------------------------------------------

function parseParameters(text) {
  const out = {};
  for (const line of text.split('\n')) {
    const index = line.indexOf('=');
    if (index > 0) out[line.slice(0, index).trim()] = line.slice(index + 1).trim();
  }
  return out;
}

async function init() {
  const behaviors = await api('/api/behaviors');
  state.behaviors = behaviors.behaviors || [];
  renderPalette('');

  $('run').addEventListener('click', async () => {
    const result = await api('/api/run', {
      method: 'POST',
      body: JSON.stringify({ name: $('objective-list').value,
                             parameters: parseParameters($('objective-parameters').value) }),
    });
    if (!result.ok) alert(result.error);
  });

  $('cancel').addEventListener('click', () => api('/api/cancel', { method: 'POST' }));

  $('teach').addEventListener('click', async () => {
    const name = $('waypoint-name').value.trim();
    if (!name) return;
    const result = await api('/api/waypoint', { method: 'POST',
                                                body: JSON.stringify({ name, overwrite: true }) });
    if (!result.ok) alert(result.error);
    else $('waypoint-name').value = '';
  });

  $('load-objective').addEventListener('click', async () => {
    const name = $('edit-objective').value;
    if (!name) return;
    const data = await api('/api/objective/' + encodeURIComponent(name));
    if (!data.found) { alert('not found'); return; }
    state.objectiveName = name;
    state.root = parseXml(data.xml);
    state.selected = null;
    state.liveStatuses = {};
    render();
    renderInspector();
    $('validation').innerHTML = '';
  });

  $('toggle-edit').addEventListener('click', () => {
    state.editing = !state.editing;
    $('toggle-edit').textContent = state.editing ? 'Stop editing' : 'Edit';
    $('edit-mode').textContent = state.editing ? 'editing' : 'watching';
    $('palette').classList.toggle('hidden', !state.editing);
    $('inspector').classList.toggle('hidden', !state.editing);
    $('validate').disabled = !state.editing;
    $('save').disabled = !state.editing;
    renderInspector();
  });

  $('validate').addEventListener('click', async () => {
    showValidation(await api('/api/validate', {
      method: 'POST', body: JSON.stringify({ xml: toXml(state.root, state.objectiveName) }),
    }));
  });

  $('save').addEventListener('click', async () => {
    // The editor emits normalised XML, so comments and hand formatting do not survive. The server
    // keeps a .bak, but saying so beforehand is the difference between a backup and a surprise.
    if (!confirm('Saving rewrites the file from the editor\'s model.\n\n' +
                 'Comments and hand formatting will be lost. The server writes a .bak first.\n\n' +
                 'Continue?')) return;
    const result = await api('/api/objective/' + encodeURIComponent(state.objectiveName), {
      method: 'POST', body: JSON.stringify({ xml: toXml(state.root, state.objectiveName) }),
    });
    showValidation(result.ok ? { valid: true } : result);
    if (result.ok) alert('saved to ' + result.written_path + '\nbackup: ' + (result.backup_path || 'none'));
  });

  $('delete-node').addEventListener('click', () => {
    if (!state.selected || !state.root) return;
    if (state.selected === state.root) { state.root = null; state.selected = null; }
    else {
      const prune = (node) => {
        node.children = node.children.filter((c) => c !== state.selected);
        node.children.forEach(prune);
      };
      prune(state.root);
      state.selected = null;
    }
    render();
    renderInspector();
  });

  $('palette-filter').addEventListener('input', (event) => renderPalette(event.target.value));

  for (const button of document.querySelectorAll('header nav button')) {
    button.addEventListener('click', () => {
      document.querySelectorAll('header nav button').forEach((b) => b.classList.remove('active'));
      button.classList.add('active');
      $('view-tree').classList.toggle('hidden', button.dataset.view !== 'tree');
      $('view-robot').classList.toggle('hidden', button.dataset.view !== 'robot');
    });
  }

  $('objective-list').addEventListener('change', () => poll());

  poll();
  setInterval(poll, 500);
}

init();
