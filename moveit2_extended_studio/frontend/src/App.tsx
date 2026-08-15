// Copyright 2026 Leow Chee Siang. Apache-2.0.
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import {
  Background,
  Controls,
  MiniMap,
  ReactFlow,
  ReactFlowProvider,
  useReactFlow,
  type Edge,
  type NodeMouseHandler,
  type OnNodeDrag,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";

import { api } from "./api";
import {
  cloneTree,
  extractHeader,
  findNode,
  findParent,
  flatten,
  freshId,
  insertChild,
  moveSibling,
  parseObjective,
  removeNode,
  reparent,
  serializeObjective,
  updateNode,
} from "./btxml";
import { layoutRuntime, layoutTree, NODE_HEIGHT, NODE_WIDTH } from "./layout";
import { BehaviorNode, type BehaviorFlowNode } from "./components/BehaviorNode";
import { Inspector } from "./components/Inspector";
import { Palette } from "./components/Palette";
import {
  CATEGORY_BY_CODE,
  STATUS_BY_CODE,
  type BehaviorInfo,
  type BtNode,
  type StudioState,
  type Validation,
} from "./types";

const nodeTypes = { behavior: BehaviorNode };
const POLL_MS = 300;

type Mode = "edit" | "watch";

export function App() {
  return (
    <ReactFlowProvider>
      <Studio />
    </ReactFlowProvider>
  );
}

function Studio() {
  const [behaviors, setBehaviors] = useState<BehaviorInfo[]>([]);
  const [state, setState] = useState<StudioState | null>(null);
  const [mode, setMode] = useState<Mode>("watch");

  const [objectiveName, setObjectiveName] = useState("");
  const [tree, setTree] = useState<BtNode | null>(null);
  const [treeId, setTreeId] = useState("");
  const [header, setHeader] = useState("");
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [dirty, setDirty] = useState(false);
  /** Folded subtrees. View state only -- it never reaches the XML, because how you are looking at
   *  a tree is not part of what the robot will do. */
  const [collapsed, setCollapsed] = useState<ReadonlySet<string>>(new Set());

  const [validation, setValidation] = useState<Validation | null>(null);
  const [notice, setNotice] = useState("");
  const [parameters, setParameters] = useState("");

  const { getIntersectingNodes, fitView } = useReactFlow();
  const laidOutInstance = useRef("");

  // --- data ------------------------------------------------------------------------------------

  useEffect(() => {
    api
      .behaviors()
      .then((payload) => setBehaviors(payload.behaviors))
      .catch(() => setNotice("could not read the Behavior list -- is the objective server running?"));
  }, []);

  useEffect(() => {
    let live = true;
    const tick = async () => {
      try {
        const next = await api.state();
        if (live) setState(next);
      } catch {
        if (live) setState(null);
      }
    };
    void tick();
    const timer = window.setInterval(() => void tick(), POLL_MS);
    return () => {
      live = false;
      window.clearInterval(timer);
    };
  }, []);

  const behaviorByName = useMemo(() => new Map(behaviors.map((b) => [b.name, b])), [behaviors]);

  const loadObjective = useCallback(async (name: string) => {
    if (!name) return;
    const payload = await api.objective(name);
    if (!payload.found) {
      setNotice(`the server has no Objective called ${name}`);
      return;
    }
    const parsed = parseObjective(payload.xml);
    setObjectiveName(name);
    setTree(parsed.root);
    setTreeId(parsed.treeId || name);
    setHeader(extractHeader(payload.xml));
    setSelectedId(parsed.root?.id ?? null);
    // A 40-node tree fits on screen only at a zoom where nothing is readable, so open it folded
    // below the second level and let the operator open the branch they care about.
    const all = flatten(parsed.root);
    setCollapsed(all.length > 14 ? new Set(deepBranches(parsed.root, 2)) : new Set());
    setValidation(null);
    setDirty(false);
    setNotice("");
    setMode("edit");
  }, []);

  // --- graph -----------------------------------------------------------------------------------

  const toggleCollapse = useCallback((id: string) => {
    setCollapsed((previous) => {
      const next = new Set(previous);
      if (next.has(id)) next.delete(id);
      else next.add(id);
      return next;
    });
  }, []);

  const editGraph = useMemo(() => {
    const placed = layoutTree(tree, collapsed);
    const nodes: BehaviorFlowNode[] = placed.map((entry) => ({
      id: entry.node.id,
      type: "behavior",
      position: { x: entry.x, y: entry.y },
      selected: entry.node.id === selectedId,
      data: {
        label: entry.node.name || entry.node.registration,
        registration: entry.node.registration,
        category: categoryOf(entry.node.registration, behaviorByName),
        order: entry.order,
        siblingCount: entry.siblingCount,
        ports: Object.entries(entry.node.attrs),
        missing: !behaviorByName.has(entry.node.registration) && behaviors.length > 0,
        hidden: entry.hidden,
        hasChildren: entry.node.children.length > 0,
        onToggle: toggleCollapse,
        id: entry.node.id,
      },
    }));
    const edges: Edge[] = placed
      .filter((entry) => entry.parentId)
      .map((entry) => ({
        id: `${entry.parentId}-${entry.node.id}`,
        source: entry.parentId as string,
        target: entry.node.id,
        type: "smoothstep",
      }));
    return { nodes, edges };
  }, [tree, selectedId, behaviorByName, behaviors.length, collapsed, toggleCollapse]);

  const watchGraph = useMemo(() => {
    const runtimeNodes = state?.tree?.nodes ?? [];
    const placed = layoutRuntime(runtimeNodes);
    const statuses = state?.statuses ?? {};
    const nodes: BehaviorFlowNode[] = placed.map((entry) => ({
      id: String(entry.uid),
      type: "behavior",
      position: { x: entry.x, y: entry.y },
      data: {
        label: entry.label,
        registration: entry.registration,
        category: CATEGORY_BY_CODE[entry.type] ?? "unknown",
        order: entry.order,
        siblingCount: 2, // always show the order badge while watching; sequence position is the story
        ports: [],
        status: STATUS_BY_CODE[statuses[String(entry.uid)] ?? 0] ?? "idle",
      },
    }));
    const edges: Edge[] = placed
      .filter((entry) => entry.parent !== null)
      .map((entry) => ({
        id: `${entry.parent}-${entry.uid}`,
        source: String(entry.parent),
        target: String(entry.uid),
        type: "smoothstep",
      }));
    return { nodes, edges };
  }, [state]);

  const graph = mode === "edit" ? editGraph : watchGraph;

  // Refit when a different tree instance starts, not on every tick -- refitting continuously
  // fights the user's own panning.
  useEffect(() => {
    const instance = `${mode}:${mode === "edit" ? objectiveName : (state?.tree?.instance ?? "")}:${graph.nodes.length}`;
    if (instance === laidOutInstance.current || graph.nodes.length === 0) return;
    laidOutInstance.current = instance;
    // Two frames, not a fixed delay: React Flow needs to have measured the nodes before it can
    // frame them, and a 50 ms guess framed an empty canvas on first load.
    requestAnimationFrame(() => requestAnimationFrame(() => fitView({ padding: 0.12, duration: 200, minZoom: 0.04 })));
  }, [mode, objectiveName, state?.tree?.instance, graph.nodes.length, fitView]);

  // --- editing ---------------------------------------------------------------------------------

  const selected = useMemo(() => findNode(tree, selectedId ?? ""), [tree, selectedId]);
  const selectedParent = useMemo(() => findParent(tree, selectedId ?? ""), [tree, selectedId]);
  const selectedOrder = selectedParent && selected ? selectedParent.children.findIndex((c) => c.id === selected.id) : 0;

  const mutate = useCallback((next: BtNode | null) => {
    setTree(next);
    setDirty(true);
    setValidation(null);
  }, []);

  const addChild = useCallback(
    (behavior: BehaviorInfo) => {
      if (!tree || !selectedId) return;
      const child: BtNode = { id: freshId(), registration: behavior.name, name: "", attrs: {}, children: [] };
      mutate(insertChild(tree, selectedId, child));
      setSelectedId(child.id);
    },
    [tree, selectedId, mutate],
  );

  const onNodeClick: NodeMouseHandler = useCallback((_event, node) => {
    setSelectedId(node.id);
  }, []);

  /**
   * Dragging re-parents; it does not move the node. Positions are computed from the tree, so a
   * free-dragged node would be drawn somewhere that contradicts the order it actually runs in.
   */
  const onNodeDragStop: OnNodeDrag<BehaviorFlowNode> = useCallback(
    (_event, node) => {
      if (mode !== "edit" || !tree) return;
      const overlapping = getIntersectingNodes({
        id: node.id,
        position: node.position,
        width: NODE_WIDTH,
        height: NODE_HEIGHT,
      });
      const target = overlapping.find((candidate) => candidate.id !== node.id);
      if (!target) {
        setTree((current) => (current ? cloneTree(current) : current)); // snap back
        return;
      }
      const next = reparent(tree, node.id, target.id);
      if (next === tree) {
        setNotice("that would put the node inside itself");
        setTree(cloneTree(tree));
        return;
      }
      setNotice("");
      mutate(next);
    },
    [mode, tree, getIntersectingNodes, mutate],
  );

  // --- server round trips ----------------------------------------------------------------------

  const xml = useMemo(() => serializeObjective(tree, treeId, header), [tree, treeId, header]);

  const validate = useCallback(async () => {
    try {
      setValidation(await api.validate(xml));
    } catch (error) {
      setNotice(String(error));
    }
  }, [xml]);

  const save = useCallback(async () => {
    try {
      const result = await api.save(objectiveName, xml);
      setValidation(result);
      if (result.ok) {
        setDirty(false);
        setNotice(`saved to ${result.written_path}${result.backup_path ? ` (backup ${result.backup_path})` : ""}`);
      } else {
        setNotice(result.error || "the server refused the save");
      }
    } catch (error) {
      setNotice(String(error));
    }
  }, [objectiveName, xml]);

  const run = useCallback(async () => {
    const values: Record<string, string> = {};
    for (const line of parameters.split("\n")) {
      const index = line.indexOf("=");
      if (index > 0) values[line.slice(0, index).trim()] = line.slice(index + 1).trim();
    }
    const target = objectiveName || state?.objectives[0]?.name || "";
    const result = await api.run(target, values);
    setNotice(result.ok ? `running ${target}` : result.error);
    if (result.ok) setMode("watch");
  }, [parameters, objectiveName, state]);

  // --- render ----------------------------------------------------------------------------------

  const objectiveState = state?.objective;
  const prompt = state?.prompt;
  const running = objectiveState?.state === "RUNNING";

  return (
    <div className="app">
      <header>
        <h1>moveit2-extended</h1>
        <span className={`chip ${(objectiveState?.state ?? "idle").toLowerCase()}`}>
          {objectiveState?.state ?? "…"}
        </span>
        <span className="current">
          {objectiveState?.objective}
          {running && objectiveState?.current_behavior ? ` · ${objectiveState.current_behavior}` : ""}
        </span>
        <nav>
          <button type="button" className={mode === "watch" ? "active" : ""} onClick={() => setMode("watch")}>
            Watch
          </button>
          <button
            type="button"
            className={mode === "edit" ? "active" : ""}
            disabled={!tree}
            onClick={() => setMode("edit")}
          >
            Edit{dirty ? " •" : ""}
          </button>
        </nav>
      </header>

      {prompt?.id && (
        <div className="prompt">
          <span>{prompt.message}</span>
          <span className="prompt-buttons">
            {(prompt.choices ?? []).map((choice) => (
              <button key={choice} type="button" onClick={() => void api.answer(prompt.id as string, choice)}>
                {choice}
              </button>
            ))}
          </span>
        </div>
      )}

      <main>
        <aside className="sidebar">
          <h2>Objectives</h2>
          <select
            size={6}
            value={objectiveName}
            onChange={(event) => void loadObjective(event.target.value)}
          >
            {(state?.objectives ?? []).map((objective) => (
              <option key={objective.name} value={objective.name} disabled={objective.missing.length > 0}>
                {objective.name}
                {objective.missing.length > 0 ? "  (unavailable)" : ""}
              </option>
            ))}
          </select>

          <label className="field">
            <span>parameters — one per line, key=value</span>
            <textarea rows={3} spellCheck={false} value={parameters} onChange={(e) => setParameters(e.target.value)} />
          </label>
          <div className="row">
            <button type="button" onClick={() => void run()} disabled={running}>
              Run
            </button>
            <button type="button" className="secondary" disabled={!running} onClick={() => void api.cancel()}>
              Cancel
            </button>
          </div>

          <h2>Log</h2>
          <div className="log">
            {(state?.log ?? []).slice(-14).map((line, index) => (
              <div key={`${index}-${line}`}>{line}</div>
            ))}
          </div>
        </aside>

        {mode === "edit" && (
          <Palette behaviors={behaviors} disabled={!selectedId} onAdd={addChild} />
        )}

        <div className="canvas">
          <ReactFlow
            nodes={graph.nodes}
            edges={graph.edges}
            nodeTypes={nodeTypes}
            onNodeClick={onNodeClick}
            onNodeDragStop={onNodeDragStop}
            nodesDraggable={mode === "edit"}
            nodesConnectable={false}
            colorMode="dark"
            // A 40-node tree is several thousand pixels wide, and React Flow's default minimum
            // zoom of 0.5 is too high to ever frame it: fitView silently clamps and the tree
            // stays cut off at both edges, looking like fitView is broken.
            minZoom={0.04}
            maxZoom={2}
            elementsSelectable
            proOptions={{ hideAttribution: true }}
            fitView
          >
            <Background gap={22} size={1} />
            <Controls showInteractive={false} />
            <MiniMap pannable zoomable />
          </ReactFlow>
          {graph.nodes.length === 0 && (
            <p className="empty">
              {mode === "edit"
                ? "Pick an Objective to edit."
                : "Nothing has run yet. Run an Objective and the tree appears here."}
            </p>
          )}
        </div>

        {mode === "edit" && (
          <Inspector
            node={selected}
            behavior={selected ? behaviorByName.get(selected.registration) : undefined}
            order={selectedOrder}
            siblingCount={selectedParent?.children.length ?? 1}
            canDelete={Boolean(selectedParent)}
            onRename={(name) => tree && selectedId && mutate(updateNode(tree, selectedId, (n) => (n.name = name)))}
            onPort={(key, value) =>
              tree &&
              selectedId &&
              mutate(
                updateNode(tree, selectedId, (n) => {
                  if (value === "") delete n.attrs[key];
                  else n.attrs[key] = value;
                }),
              )
            }
            onMove={(delta) => tree && selectedId && mutate(moveSibling(tree, selectedId, delta))}
            onDelete={() => {
              if (!tree || !selectedId) return;
              const parent = findParent(tree, selectedId);
              mutate(removeNode(tree, selectedId));
              setSelectedId(parent?.id ?? null);
            }}
          />
        )}
      </main>

      <footer>
        {mode === "edit" ? (
          <>
            <button type="button" className="secondary" onClick={() => void validate()}>
              Validate
            </button>
            <button type="button" onClick={() => void save()} disabled={!dirty}>
              Save
            </button>
            <ValidationReport validation={validation} nodeCount={flatten(tree).length} />
          </>
        ) : (
          <span className="muted small">
            {state?.tree?.objective ? `${state.tree.objective} · ${graph.nodes.length} nodes` : "no tree yet"}
          </span>
        )}
        {notice && <span className="notice">{notice}</span>}
      </footer>
    </div>
  );
}

function ValidationReport({ validation, nodeCount }: { validation: Validation | null; nodeCount: number }) {
  if (!validation) return <span className="muted small">{nodeCount} nodes · not validated yet</span>;
  if (validation.valid) return <span className="ok small">valid · {nodeCount} nodes</span>;

  // Four separate lists, because each one needs a different fix.
  const buckets: [string, string[]][] = [
    ["XML", validation.xml_error ? [validation.xml_error] : []],
    ["no such Behavior", validation.missing_behaviors],
    ["no such port", validation.unknown_ports],
    ["bad value", validation.invalid_port_values],
  ];
  return (
    <span className="bad small">
      {buckets
        .filter(([, list]) => list.length > 0)
        .map(([label, list]) => `${label}: ${list.join(", ")}`)
        .join("  |  ")}
    </span>
  );
}

/** Ids of every node at or below `depth` that has children -- the branches to fold on open. */
function deepBranches(root: BtNode | null, depth: number): string[] {
  const ids: string[] = [];
  const walk = (node: BtNode, level: number) => {
    if (level >= depth && node.children.length > 0) {
      ids.push(node.id);
      return; // no need to fold inside something already folded
    }
    for (const child of node.children) walk(child, level + 1);
  };
  if (root) walk(root, 0);
  return ids;
}

function categoryOf(registration: string, behaviors: Map<string, BehaviorInfo>) {
  const known = behaviors.get(registration);
  if (known) return CATEGORY_BY_CODE[known.type] ?? "unknown";
  // BehaviorTree.CPP's own control and decorator nodes are not plugins, so they are not in the
  // list the server returns; recognise the common ones so the graph is not all one colour.
  if (/^(Sequence|SequenceStar|ReactiveSequence|Fallback|ReactiveFallback|Parallel|IfThenElse|WhileDoElse|Switch)/.test(registration))
    return "control";
  if (/^(Inverter|ForceSuccess|ForceFailure|Repeat|RetryUntilSuccessful|Timeout|Delay|KeepRunningUntilFailure|RunOnce)/.test(registration))
    return "decorator";
  if (/^SubTree/.test(registration)) return "subtree";
  return "unknown";
}
