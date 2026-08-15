// Copyright 2026 Leow Chee Siang. Apache-2.0.
//
// Typed client for studio_server's JSON API. Every call is one round trip and returns plain data;
// there is deliberately no caching layer, because the interesting state (what the tree is doing)
// changes several times a second and a stale cache of it would be worse than no UI at all.

import type { BehaviorInfo, SaveResult, StudioState, Validation } from "./types";

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(path, {
    ...init,
    headers: init?.body ? { "Content-Type": "application/json" } : undefined,
  });
  if (!response.ok) throw new Error(`${path}: HTTP ${response.status}`);
  return (await response.json()) as T;
}

export const api = {
  state: () => request<StudioState>("/api/state"),

  behaviors: () => request<{ behaviors: BehaviorInfo[] }>("/api/behaviors"),

  objective: (name: string) =>
    request<{ found: boolean; xml: string; description?: string }>(`/api/objective/${encodeURIComponent(name)}`),

  /** Server-side validation. Four separate answers, kept separate, because they need different fixes. */
  validate: (xml: string) =>
    request<Validation>("/api/validate", { method: "POST", body: JSON.stringify({ xml }) }),

  /** Saves only after the server has validated; the server keeps a .bak of what was there. */
  save: (name: string, xml: string) =>
    request<SaveResult>(`/api/objective/${encodeURIComponent(name)}`, {
      method: "POST",
      body: JSON.stringify({ xml }),
    }),

  run: (name: string, parameters: Record<string, string>) =>
    request<{ ok: boolean; error: string }>("/api/run", {
      method: "POST",
      body: JSON.stringify({ name, parameters }),
    }),

  cancel: () => request<{ ok: boolean; error: string }>("/api/cancel", { method: "POST" }),

  answer: (id: string, choice: string) =>
    request<{ ok: boolean }>("/api/answer", { method: "POST", body: JSON.stringify({ id, choice }) }),

  tool: (name: string) =>
    request<{ ok: boolean; error: string }>("/api/tool", { method: "POST", body: JSON.stringify({ name }) }),

  waypoint: (body: { action: "teach" | "delete"; name: string; overwrite?: boolean; tags?: string }) =>
    request<{ ok: boolean; error?: string }>("/api/waypoint", { method: "POST", body: JSON.stringify(body) }),
};
