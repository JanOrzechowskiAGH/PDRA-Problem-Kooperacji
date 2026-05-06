from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from des_model import (
    Event,
    EventKind,
    Supervisor,
    TeamState,
    admissible_events_for_robot,
    all_points_explored,
    build_adj,
    default_edges,
    initial_team_state,
    step_robot,
    zone_of_point,
)


def short_state_label(team: TeamState) -> str:
    """
    Keep nodes readable by projecting the full team state to:
      (pos1, pos2, explored_zones, active flags)
    """
    ez = "".join(sorted(z.value for z in team.explored_zones)) or "∅"
    a1 = "1" if team.r1.active else "0"
    a2 = "1" if team.r2.active else "0"
    return f"({team.r1.pos},{team.r2.pos})|E:{ez}|act:{a1}{a2}"


def state_key(team: TeamState) -> Tuple:
    return (
        team.r1.pos,
        tuple(sorted(team.r1.explored_points)),
        team.r1.active,
        team.r2.pos,
        tuple(sorted(team.r2.explored_points)),
        team.r2.active,
        tuple(sorted(z.value for z in team.explored_zones)),
    )


def enabled_events(
    team: TeamState,
    adj,
    supervisor: Optional[Supervisor],
) -> Tuple[List[Event], List[Event]]:
    adm1 = admissible_events_for_robot(team, 1, adj)
    adm2 = admissible_events_for_robot(team, 2, adj)
    if supervisor is None:
        return adm1, adm2
    return supervisor.enabled(team, 1, adm1), supervisor.enabled(team, 2, adm2)


def to_dot(nodes: Dict[Tuple, str], edges: List[Tuple[str, str, str]]) -> str:
    lines: List[str] = []
    lines.append("digraph TeamDES {")
    lines.append('  rankdir="LR";')
    lines.append('  node [shape=box, fontname="Helvetica", fontsize=10];')
    lines.append('  edge [fontname="Helvetica", fontsize=9];')

    # nodes
    for _, nid in nodes.items():
        label = nid
        # escape quotes
        label = label.replace('"', '\\"')
        lines.append(f'  "{nid}" [label="{label}"];')

    # edges
    for u, v, lab in edges:
        lab = lab.replace('"', '\\"')
        lines.append(f'  "{u}" -> "{v}" [label="{lab}"];')

    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="out/team_graph.dot")
    ap.add_argument("--max-nodes", type=int, default=500)
    ap.add_argument("--max-edges", type=int, default=2000)
    ap.add_argument("--use-supervisor", action="store_true")
    args = ap.parse_args()

    adj = build_adj(default_edges())
    sup = Supervisor() if args.use_supervisor else None

    start = initial_team_state()
    q = deque([start])

    nodes: Dict[Tuple, str] = {}
    edges: List[Tuple[str, str, str]] = []

    def ensure_node(team: TeamState) -> str:
        k = state_key(team)
        if k not in nodes:
            nodes[k] = short_state_label(team)
        return nodes[k]

    ensure_node(start)

    seen: Set[Tuple] = {state_key(start)}

    while q and len(nodes) < args.max_nodes and len(edges) < args.max_edges:
        cur = q.popleft()
        cur_id = ensure_node(cur)

        e1s, e2s = enabled_events(cur, adj, sup)

        # asynchronous event semantics: allow one robot event at a time
        candidates: List[Tuple[Optional[Event], Optional[Event], str]] = []
        for e in e1s:
            candidates.append((e, None, e.label()))
        for e in e2s:
            candidates.append((None, e, e.label()))

        # also allow a simultaneous pair if they don't trivially conflict
        for e1 in e1s:
            for e2 in e2s:
                if e1.kind == EventKind.ENTER and e2.kind == EventKind.ENTER and e1.arg == e2.arg and e1.arg != "x":
                    continue
                candidates.append((e1, e2, f"{e1.label()} || {e2.label()}"))

        for a1, a2, lab in candidates:
            nxt = cur
            try:
                if a1 is not None:
                    nxt, _, _ = step_robot(nxt, a1, adj)
                if a2 is not None:
                    nxt, _, _ = step_robot(nxt, a2, adj)
            except Exception:
                continue

            nxt_id = ensure_node(nxt)
            edges.append((cur_id, nxt_id, lab))

            k2 = state_key(nxt)
            if k2 not in seen and len(nodes) < args.max_nodes:
                seen.add(k2)
                q.append(nxt)

    dot = to_dot(nodes, edges)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(dot)

    print(f"Wrote {args.out} with {len(nodes)} nodes, {len(edges)} edges.")


if __name__ == "__main__":
    main()

