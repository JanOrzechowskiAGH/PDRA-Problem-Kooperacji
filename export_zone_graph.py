from __future__ import annotations

import argparse
import csv
import os
from collections import deque
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

from des_model import EventKind, Zone, build_adj, default_edges


ZonePair = Tuple[Zone, Zone]


def zone_adj() -> Dict[Zone, Set[Zone]]:
    """
    Derive zone-to-zone adjacency from inter-zone edges in the default graph.
    """
    adj = build_adj(default_edges())
    out: Dict[Zone, Set[Zone]] = {z: set() for z in (Zone.X, Zone.A, Zone.B, Zone.C)}

    def z_of_point(p: str) -> Zone:
        if p == "x":
            return Zone.X
        return Zone(p[0])

    for u, edges in adj.items():
        zu = z_of_point(u)
        for e in edges:
            if not e.inter_zone:
                continue
            zv = z_of_point(e.v)
            out[zu].add(zv)
    return out


def parse_solution_zone_edges(pareto_csv_path: str) -> Tuple[Set[ZonePair], Set[Tuple[ZonePair, ZonePair]]]:
    """
    Extract zone-pair nodes and edges used by Pareto trajectories.

    We track only ENTER/LEAVE events; MOVE/EXPLORE don't change the zone.
    """
    used_nodes: Set[ZonePair] = set()
    used_edges: Set[Tuple[ZonePair, ZonePair]] = set()

    with open(pareto_csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            traj = row["trajectory"]
            # trajectory format: "<tag> :: ev | ev | ..."
            parts = traj.split("::", 1)
            evs = parts[1] if len(parts) == 2 else parts[0]
            ev_list = [p.strip() for p in evs.split("|")]

            z1 = Zone.X
            z2 = Zone.X
            prev = (z1, z2)
            used_nodes.add(prev)

            for ev in ev_list:
                # examples: "R1:enter(A)" or "R2:leave(x)" or "R1:finish"
                if ev.startswith("R1:enter("):
                    tgt = ev[len("R1:enter(") : -1]
                    z1 = Zone(tgt)
                elif ev.startswith("R2:enter("):
                    tgt = ev[len("R2:enter(") : -1]
                    z2 = Zone(tgt)
                elif ev.startswith("R1:leave("):
                    z1 = Zone.X
                elif ev.startswith("R2:leave("):
                    z2 = Zone.X
                else:
                    continue

                cur = (z1, z2)
                used_nodes.add(cur)
                used_edges.add((prev, cur))
                prev = cur

    return used_nodes, used_edges


def bfs_zone_pairs(max_nodes: int = 100) -> Tuple[List[ZonePair], List[Tuple[ZonePair, ZonePair, str]]]:
    """
    Build a zone-only product graph assuming:
    - robots can change only their own zone at a time,
    - a simultaneous change is also allowed (will be emitted as a separate edge label),
    - zone-exclusivity is enforced (no two robots in same zone A/B/C).
    """
    zadj = zone_adj()
    start: ZonePair = (Zone.X, Zone.X)
    q = deque([start])
    seen: Set[ZonePair] = {start}
    nodes: List[ZonePair] = []
    edges: List[Tuple[ZonePair, ZonePair, str]] = []

    def ok(pair: ZonePair) -> bool:
        a, b = pair
        if a == b and a in (Zone.A, Zone.B, Zone.C):
            return False
        return True

    while q and len(seen) < max_nodes:
        cur = q.popleft()
        nodes.append(cur)
        z1, z2 = cur

        # single-robot changes
        for nz1 in zadj[z1]:
            nxt = (nz1, z2)
            if ok(nxt):
                edges.append((cur, nxt, f"R1:{z1.value}→{nz1.value}"))
                if nxt not in seen:
                    seen.add(nxt)
                    q.append(nxt)

        for nz2 in zadj[z2]:
            nxt = (z1, nz2)
            if ok(nxt):
                edges.append((cur, nxt, f"R2:{z2.value}→{nz2.value}"))
                if nxt not in seen:
                    seen.add(nxt)
                    q.append(nxt)

        # simultaneous change (optional)
        for nz1 in zadj[z1]:
            for nz2 in zadj[z2]:
                nxt = (nz1, nz2)
                if ok(nxt):
                    edges.append((cur, nxt, f"R1:{z1.value}→{nz1.value} || R2:{z2.value}→{nz2.value}"))
                    if nxt not in seen:
                        seen.add(nxt)
                        q.append(nxt)

    return nodes, edges


def node_id(p: ZonePair) -> str:
    return f"({p[0].value},{p[1].value})"


def to_dot(
    nodes: List[ZonePair],
    edges: List[Tuple[ZonePair, ZonePair, str]],
    highlight_nodes: Set[ZonePair],
    highlight_edges: Set[Tuple[ZonePair, ZonePair]],
) -> str:
    lines: List[str] = []
    lines.append("digraph TeamZones {")
    lines.append('  rankdir="LR";')
    lines.append('  node [shape=circle, fontname="Helvetica", fontsize=11, width=1.0];')
    lines.append('  edge [fontname="Helvetica", fontsize=9];')

    for n in nodes:
        nid = node_id(n)
        attrs: List[str] = [f'label="{nid}"']
        if n in highlight_nodes:
            attrs.append('color="red"')
            attrs.append('penwidth=2.5')
            attrs.append('fontcolor="red"')
        lines.append(f'  "{nid}" [{", ".join(attrs)}];')

    for u, v, lab in edges:
        uid = node_id(u)
        vid = node_id(v)
        attrs: List[str] = [f'label="{lab}"']
        if (u, v) in highlight_edges:
            attrs.append('color="red"')
            attrs.append("penwidth=2.5")
            attrs.append('fontcolor="red"')
        lines.append(f'  "{uid}" -> "{vid}" [{", ".join(attrs)}];')

    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pareto", default="out/pareto_candidates.csv")
    ap.add_argument("--dot-out", default="out/team_zone_graph.dot")
    ap.add_argument("--max-nodes", type=int, default=50)
    args = ap.parse_args()

    nodes, edges = bfs_zone_pairs(max_nodes=args.max_nodes)
    h_nodes, h_edges = parse_solution_zone_edges(args.pareto)

    dot = to_dot(nodes, edges, h_nodes, h_edges)
    os.makedirs(os.path.dirname(args.dot_out) or ".", exist_ok=True)
    with open(args.dot_out, "w", encoding="utf-8") as f:
        f.write(dot)

    print(f"Wrote {args.dot_out} (nodes={len(nodes)}, edges={len(edges)}), highlighted nodes={len(h_nodes)} edges={len(h_edges)}")


if __name__ == "__main__":
    main()

