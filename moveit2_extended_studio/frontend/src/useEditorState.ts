// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Undo/redo and the clipboard, kept out of the component.
//
// An editor without undo is a demo. Every edit here replaces the whole tree -- the operations in
// btxml.ts are already immutable -- so history is just a stack of those trees, which is both the
// simplest correct implementation and the cheapest: a Behavior Tree is a few hundred small objects,
// not a document.

import { useCallback, useRef, useState } from "react";
import { cloneTree, freshId } from "./btxml";
import type { BtNode } from "./types";

const HISTORY_LIMIT = 100;

export interface EditorHistory {
  tree: BtNode | null;
  canUndo: boolean;
  canRedo: boolean;
  /** Replace the tree and push the previous one onto the undo stack. */
  commit: (next: BtNode | null) => void;
  /** Replace the tree without touching history -- for loading a different Objective. */
  reset: (next: BtNode | null) => void;
  undo: () => void;
  redo: () => void;
  /** Increments on every history change.
   *
   *  canUndo/canRedo are read off refs, and a ref changing does not re-render -- so without a
   *  state that also changes, the undo button would stay greyed out after the first edit. This is
   *  that state, exposed rather than hidden so the reason it exists is visible. */
  version: number;
}

export function useTreeHistory(): EditorHistory {
  const [tree, setTree] = useState<BtNode | null>(null);
  const past = useRef<(BtNode | null)[]>([]);
  const future = useRef<(BtNode | null)[]>([]);
  const [version, setVersion] = useState(0);

  const commit = useCallback((next: BtNode | null) => {
    setTree((current) => {
      past.current.push(current);
      if (past.current.length > HISTORY_LIMIT) past.current.shift();
      // A new edit invalidates anything that was undone: the branch it belonged to is gone.
      future.current = [];
      return next;
    });
    setVersion((v) => v + 1);
  }, []);

  const reset = useCallback((next: BtNode | null) => {
    past.current = [];
    future.current = [];
    setTree(next);
    setVersion((v) => v + 1);
  }, []);

  const undo = useCallback(() => {
    setTree((current) => {
      const previous = past.current.pop();
      if (previous === undefined) return current;
      future.current.push(current);
      return previous;
    });
    setVersion((v) => v + 1);
  }, []);

  const redo = useCallback(() => {
    setTree((current) => {
      const next = future.current.pop();
      if (next === undefined) return current;
      past.current.push(current);
      return next;
    });
    setVersion((v) => v + 1);
  }, []);

  return {
    tree,
    canUndo: past.current.length > 0,
    canRedo: future.current.length > 0,
    commit,
    reset,
    undo,
    redo,
    version,
  };
}

/** A subtree copied out of the document, with fresh ids so pasting it twice makes two nodes. */
export function withFreshIds(node: BtNode): BtNode {
  const copy = cloneTree(node);
  const renumber = (target: BtNode) => {
    target.id = freshId();
    target.children.forEach(renumber);
  };
  renumber(copy);
  return copy;
}
