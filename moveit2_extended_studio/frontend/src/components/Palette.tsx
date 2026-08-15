// Copyright 2026 Leow Chee Siang. Apache-2.0.
import { useMemo, useState } from "react";
import type { BehaviorInfo } from "../types";
import { CATEGORY_BY_CODE } from "../types";

interface Props {
  behaviors: BehaviorInfo[];
  disabled: boolean;
  onAdd: (behavior: BehaviorInfo) => void;
}

/**
 * The palette groups by the package that registered each Behavior, because that is the axis that
 * actually helps: "which of my packages provides this" is the question asked when a Behavior is
 * missing on the robot, and control-flow nodes come from BehaviorTree.CPP itself.
 */
export function Palette({ behaviors, disabled, onAdd }: Props) {
  const [filter, setFilter] = useState("");

  const groups = useMemo(() => {
    const needle = filter.trim().toLowerCase();
    const matching = needle
      ? behaviors.filter(
          (b) => b.name.toLowerCase().includes(needle) || b.description.toLowerCase().includes(needle),
        )
      : behaviors;
    const byPackage = new Map<string, BehaviorInfo[]>();
    for (const behavior of matching) {
      const key = behavior.package || "builtin";
      const list = byPackage.get(key);
      if (list) list.push(behavior);
      else byPackage.set(key, [behavior]);
    }
    return [...byPackage.entries()].sort(([a], [b]) => a.localeCompare(b));
  }, [behaviors, filter]);

  const total = groups.reduce((sum, [, list]) => sum + list.length, 0);

  return (
    <aside className="palette">
      <input
        className="filter"
        placeholder={`filter ${behaviors.length} behaviors`}
        value={filter}
        spellCheck={false}
        onChange={(event) => setFilter(event.target.value)}
      />
      {filter && <div className="muted small">{total} match</div>}

      <div className="palette-list">
        {groups.map(([pkg, list]) => (
          <section key={pkg}>
            <h4>{pkg}</h4>
            {list.map((behavior) => (
              <button
                key={behavior.name}
                type="button"
                className={`palette-item cat-${CATEGORY_BY_CODE[behavior.type] ?? "unknown"}`}
                disabled={disabled}
                title={behavior.description || behavior.name}
                onClick={() => onAdd(behavior)}
              >
                <span className="palette-name">{behavior.name}</span>
                <span className="palette-ports">{behavior.ports.length || ""}</span>
              </button>
            ))}
          </section>
        ))}
      </div>
      {disabled && <p className="muted small">Select a node to add a child to it.</p>}
    </aside>
  );
}
