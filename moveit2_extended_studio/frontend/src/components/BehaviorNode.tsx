// Copyright 2026 Leow Chee Siang. Apache-2.0.
import { Handle, Position, type NodeProps, type Node } from "@xyflow/react";
import type { BtCategory, BtStatus } from "../types";
import { blackboardKey } from "../btxml";

export interface BehaviorNodeData extends Record<string, unknown> {
  label: string;
  registration: string;
  category: BtCategory;
  /** Index among siblings. Shown because in a Behavior Tree this is the execution order. */
  order: number;
  siblingCount: number;
  ports: [string, string][];
  status?: BtStatus;
  selected?: boolean;
  /** Set when the server says no plugin registered this name. */
  missing?: boolean;
  /** Descendants folded away under this node; 0 when it is expanded or a leaf. */
  hidden?: number;
  /** Ports that exist but are not drawn, because the port-count limit cut them off. */
  hiddenPorts?: number;
  /** Matches the current search. */
  match?: boolean;
  /** IS the current search match, i.e. the one navigation is sitting on. */
  current?: boolean;
  /** Shares a blackboard key with the selected node -- the data-flow highlight. */
  inFlow?: boolean;
  hasChildren?: boolean;
  onToggle?: (id: string) => void;
  id?: string;
}

export type BehaviorFlowNode = Node<BehaviorNodeData, "behavior">;

export function BehaviorNode({ data }: NodeProps<BehaviorFlowNode>) {
  const classes = [
    "bt-node",
    `cat-${data.category}`,
    data.status ? `st-${data.status}` : "",
    data.missing ? "missing" : "",
    data.match ? "match" : "",
    data.current ? "current-match" : "",
    data.inFlow ? "in-flow" : "",
  ]
    .filter(Boolean)
    .join(" ");

  return (
    <div className={classes} title={data.missing ? "no plugin registered this Behavior" : data.registration}>
      <Handle type="target" position={Position.Top} />

      <div className="bt-node-head">
        {data.siblingCount > 1 && (
          <span className="bt-order" title={`runs ${ordinal(data.order + 1)} among its siblings`}>
            {data.order + 1}
          </span>
        )}
        <span className="bt-name">{data.label}</span>
        {data.hasChildren && data.onToggle && data.id && (
          <button
            type="button"
            className="bt-fold"
            title={data.hidden ? `expand ${data.hidden} hidden` : "collapse"}
            onClick={(event) => {
              // Stop the canvas from also treating this as a node selection/drag.
              event.stopPropagation();
              data.onToggle?.(data.id as string);
            }}
          >
            {data.hidden ? `+${data.hidden}` : "−"}
          </button>
        )}
      </div>

      <div className="bt-reg">{data.registration}</div>

      {data.ports.length > 0 && (
        <div className="bt-ports">
          {data.ports.map(([key, value]) => (
            <span key={key} className="bt-port">
              <span className="bt-port-key">{key}</span>
              <span className={blackboardKey(value) ? "bt-port-ref" : "bt-port-val"}>{shorten(value)}</span>
            </span>
          ))}
          {(data.hiddenPorts ?? 0) > 0 && <span className="bt-port more">+{data.hiddenPorts}</span>}
        </div>
      )}

      <Handle type="source" position={Position.Bottom} />
    </div>
  );
}

function shorten(value: string): string {
  return value.length > 18 ? `${value.slice(0, 17)}…` : value;
}

function ordinal(n: number): string {
  const suffix = n % 10 === 1 && n % 100 !== 11 ? "st" : n % 10 === 2 && n % 100 !== 12 ? "nd" : n % 10 === 3 && n % 100 !== 13 ? "rd" : "th";
  return `${n}${suffix}`;
}
