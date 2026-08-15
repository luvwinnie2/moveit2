// Copyright 2026 Leow Chee Siang. Apache-2.0.
import type { BehaviorInfo, BtNode } from "../types";
import { blackboardKey } from "../btxml";

interface Props {
  node: BtNode | null;
  behavior: BehaviorInfo | undefined;
  order: number;
  siblingCount: number;
  canDelete: boolean;
  onRename: (name: string) => void;
  onPort: (key: string, value: string) => void;
  onMove: (delta: number) => void;
  onDelete: () => void;
}

export function Inspector({
  node,
  behavior,
  order,
  siblingCount,
  canDelete,
  onRename,
  onPort,
  onMove,
  onDelete,
}: Props) {
  if (!node) {
    return (
      <aside className="inspector">
        <p className="muted">Select a node to edit it.</p>
      </aside>
    );
  }

  const declared = behavior?.ports ?? [];
  const declaredNames = new Set(declared.map((p) => p.name));
  // Ports written in the XML that this Behavior does not declare. The server reports these too,
  // but showing them next to the value is what makes the typo obvious.
  const undeclared = Object.keys(node.attrs).filter((key) => !declaredNames.has(key));

  return (
    <aside className="inspector">
      <h3>{node.registration}</h3>
      {behavior ? (
        <p className="muted small">{behavior.description || "No description."}</p>
      ) : (
        <p className="bad small">No plugin registered this Behavior. The tree will not build.</p>
      )}

      <label className="field">
        <span>name</span>
        <input
          value={node.name}
          spellCheck={false}
          placeholder={node.registration}
          onChange={(event) => onRename(event.target.value)}
        />
      </label>

      {siblingCount > 1 && (
        <div className="order-row">
          <span className="muted small">
            runs {order + 1} of {siblingCount}
          </span>
          <span className="order-buttons">
            <button type="button" disabled={order === 0} onClick={() => onMove(-1)} title="run earlier">
              ↑
            </button>
            <button
              type="button"
              disabled={order >= siblingCount - 1}
              onClick={() => onMove(1)}
              title="run later"
            >
              ↓
            </button>
          </span>
        </div>
      )}

      {declared.length > 0 && <h4>Ports</h4>}
      {declared.map((port) => {
        const value = node.attrs[port.name] ?? "";
        const reference = blackboardKey(value);
        return (
          <label className="field" key={port.name}>
            <span>
              {port.name}
              <em className="porttype">
                {port.direction}
                {port.type ? ` · ${port.type}` : ""}
              </em>
            </span>
            <input
              className={reference ? "is-ref" : ""}
              value={value}
              spellCheck={false}
              placeholder={port.default || (port.direction === "output" ? "{blackboard_key}" : "")}
              title={port.description}
              onChange={(event) => onPort(port.name, event.target.value)}
            />
            {reference && <em className="hint">blackboard: {reference}</em>}
          </label>
        );
      })}

      {undeclared.length > 0 && (
        <>
          <h4 className="bad">Not declared by this Behavior</h4>
          {undeclared.map((key) => (
            <label className="field" key={key}>
              <span className="bad">{key}</span>
              <input value={node.attrs[key] ?? ""} spellCheck={false} onChange={(e) => onPort(key, e.target.value)} />
              <em className="hint">Clear the value to drop this attribute.</em>
            </label>
          ))}
        </>
      )}

      <div className="row">
        <button type="button" className="danger" disabled={!canDelete} onClick={onDelete}>
          Delete node and its children
        </button>
      </div>
    </aside>
  );
}
