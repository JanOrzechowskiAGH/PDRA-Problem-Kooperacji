from __future__ import annotations

import argparse
import csv
import os
from dataclasses import dataclass
from itertools import permutations
from typing import Dict, List, Sequence, Tuple

import matplotlib.pyplot as plt

from des_model import (
    Event,
    EventKind,
    TeamState,
    all_points_explored,
    build_adj,
    default_edges,
    initial_team_state,
    penalty_for_not_at_x,
    step_robot,
)


@dataclass(frozen=True)
class Eval:
    T: float
    C: float
    T1: float
    T2: float
    C1: float
    C2: float
    final_r1: str
    final_r2: str
    traj: str


def dominates(a: Eval, b: Eval) -> bool:
    # Pareto on (T,C)
    return (a.T <= b.T and a.C <= b.C) and (a.T < b.T or a.C < b.C)


def pareto(evals: List[Eval]) -> List[Eval]:
    out: List[Eval] = []
    for e in evals:
        if any(dominates(o, e) for o in evals if o is not e):
            continue
        out.append(e)
    return sorted(out, key=lambda x: (x.T, x.C))


def simulate_interleaving(seq1: Sequence[Event], seq2: Sequence[Event]) -> Tuple[TeamState, float, float, float, float, float, List[str]]:
    """
    Simulate two robots executing their event sequences with maximal concurrency:
    - when both have a next event, start both at the same global time
      (sequentially applied with plant constraints).
    - global time advances by min remaining duration, like a two-server timed DES.

    This is intentionally *restricted* to the candidate sequences we generate
    (which avoid collisions and opposite-direction inter-zone conflicts).
    """
    adj = build_adj(default_edges())
    team = initial_team_state()

    i1 = i2 = 0
    r1_rem = r2_rem = 0.0
    T = 0.0
    T1 = T2 = 0.0
    C1 = C2 = 0.0
    labels: List[str] = []

    while True:
        # start next events if free
        started = False
        if r1_rem <= 1e-9 and i1 < len(seq1):
            ev = seq1[i1]
            team, sc, _ = step_robot(team, ev, adj)
            r1_rem = sc.dt
            T1 += sc.dt
            C1 += sc.dc
            labels.append(ev.label())
            i1 += 1
            started = True
        if r2_rem <= 1e-9 and i2 < len(seq2):
            ev = seq2[i2]
            team, sc, _ = step_robot(team, ev, adj)
            r2_rem = sc.dt
            T2 += sc.dt
            C2 += sc.dc
            labels.append(ev.label())
            i2 += 1
            started = True

        if i1 >= len(seq1) and i2 >= len(seq2) and r1_rem <= 1e-9 and r2_rem <= 1e-9:
            break

        # if nothing could start, advance time until someone becomes free
        if (r1_rem > 1e-9) or (r2_rem > 1e-9):
            dt_candidates = [x for x in (r1_rem, r2_rem) if x > 1e-9]
            dt = min(dt_candidates) if dt_candidates else 0.0
            T += dt
            r1_rem = max(0.0, r1_rem - dt)
            r2_rem = max(0.0, r2_rem - dt)
        else:
            # deadlock in candidate construction (should not happen)
            raise RuntimeError("No events remaining but not finished")

    return team, T, T1, T2, C1, C2, labels


def zone_events(robot: int, zone: str, visit_order: Sequence[str]) -> List[Event]:
    """
    Build a simple within-zone plan:
    - assumes robot is already at the first point in visit_order
    - explores each point; moves along the chain/triangle using MOVE events
    """
    evs: List[Event] = []
    for idx, p in enumerate(visit_order):
        evs.append(Event(robot, EventKind.EXPLORE, p))
        if idx + 1 < len(visit_order):
            evs.append(Event(robot, EventKind.MOVE, visit_order[idx + 1]))
    return evs


def generate_candidates() -> List[Tuple[List[Event], List[Event], str]]:
    """
    Candidate cooperation patterns (tractable but still systematic):
    - Split A/B in parallel (R1->A, R2->B) OR swapped
    - Then exactly one robot explores C via its zone's portal and finishes
    - The other robot optionally returns to x and finishes (or finishes in place)

    For each zone exploration we enumerate all 3! point-visit permutations.
    """
    candidates: List[Tuple[List[Event], List[Event], str]] = []

    A_pts = ["A1", "A2", "A3"]
    B_pts = ["B1", "B2", "B3"]
    # for C we assume entry at C2 if coming from A2, or at C3 if coming from B2, etc.
    # We'll generate two entry variants: via A2->C2 or via B2->C3
    C_from_A = ["C2", "C3", "C1"]
    C_from_B = ["C3", "C2", "C1"]

    for split in ["R1:A_R2:B", "R1:B_R2:A"]:
        for permA in permutations(A_pts):
            for permB in permutations(B_pts):
                # Build initial split: enter zone then move to first visit point if needed
                if split == "R1:A_R2:B":
                    r1_zone = "A"
                    r2_zone = "B"
                    r1_perm = permA
                    r2_perm = permB
                else:
                    r1_zone = "B"
                    r2_zone = "A"
                    r1_perm = permB
                    r2_perm = permA

                r1: List[Event] = [Event(1, EventKind.ENTER, r1_zone)]
                r2: List[Event] = [Event(2, EventKind.ENTER, r2_zone)]

                # Ensure starting at the first point in permutation (portal is Z1)
                if r1_perm[0] != f"{r1_zone}1":
                    r1.append(Event(1, EventKind.MOVE, r1_perm[0]))
                if r2_perm[0] != f"{r2_zone}1":
                    r2.append(Event(2, EventKind.MOVE, r2_perm[0]))

                r1 += zone_events(1, r1_zone, r1_perm)
                r2 += zone_events(2, r2_zone, r2_perm)

                # choose who explores C
                for who in [1, 2]:
                    r1c = list(r1)
                    r2c = list(r2)
                    tag = f"{split} + R{who}:C"

                    if who == 1:
                        # move to portal point for entering C
                        # from A/B, best is via point 2 -> C2/C3
                        if r1_zone == "A":
                            if r1_perm[-1] != "A2":
                                r1c.append(Event(1, EventKind.MOVE, "A2"))
                            r1c.append(Event(1, EventKind.ENTER, "C"))
                            r1c += zone_events(1, "C", C_from_A)
                        else:
                            if r1_perm[-1] != "B2":
                                r1c.append(Event(1, EventKind.MOVE, "B2"))
                            r1c.append(Event(1, EventKind.ENTER, "C"))
                            r1c += zone_events(1, "C", C_from_B)
                        # finishing is appended only after global exploration completes
                    else:
                        if r2_zone == "A":
                            if r2_perm[-1] != "A2":
                                r2c.append(Event(2, EventKind.MOVE, "A2"))
                            r2c.append(Event(2, EventKind.ENTER, "C"))
                            r2c += zone_events(2, "C", C_from_A)
                        else:
                            if r2_perm[-1] != "B2":
                                r2c.append(Event(2, EventKind.MOVE, "B2"))
                            r2c.append(Event(2, EventKind.ENTER, "C"))
                            r2c += zone_events(2, "C", C_from_B)
                        # finishing is appended only after global exploration completes

                    candidates.append((r1c, r2c, tag))

    return candidates


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=str, default="out")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    evals: List[Eval] = []
    for r1_seq, r2_seq, tag in generate_candidates():
        team, T, T1, T2, C1, C2, labels = simulate_interleaving(r1_seq, r2_seq)
        if not all_points_explored(team):
            # candidate did not complete exploration (should be rare; skip)
            continue
        # Now both robots may legally finish (stop) anywhere.
        adj = build_adj(default_edges())
        if team.r1.active:
            team, _, _ = step_robot(team, Event(1, EventKind.FINISH, None), adj)
            labels.append("R1:finish")
        if team.r2.active:
            team, _, _ = step_robot(team, Event(2, EventKind.FINISH, None), adj)
            labels.append("R2:finish")
        pen = penalty_for_not_at_x(team)
        C = (C1 + C2) + pen
        evals.append(
            Eval(
                T=T,
                C=C,
                T1=T1,
                T2=T2,
                C1=C1,
                C2=C2,
                final_r1=team.r1.pos,
                final_r2=team.r2.pos,
                traj=f"{tag} :: " + " | ".join(labels),
            )
        )

    nd = pareto(evals)

    all_path = os.path.join(args.out, "candidates.csv")
    with open(all_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["T", "C", "T1", "T2", "C1", "C2", "final_r1", "final_r2", "trajectory"])
        for e in sorted(evals, key=lambda x: (x.T, x.C)):
            w.writerow([e.T, e.C, e.T1, e.T2, e.C1, e.C2, e.final_r1, e.final_r2, e.traj])

    nd_path = os.path.join(args.out, "pareto_candidates.csv")
    with open(nd_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["T", "C", "T1", "T2", "C1", "C2", "final_r1", "final_r2", "trajectory"])
        for e in nd:
            w.writerow([e.T, e.C, e.T1, e.T2, e.C1, e.C2, e.final_r1, e.final_r2, e.traj])

    if nd:
        plt.figure(figsize=(7, 5))
        plt.scatter([e.T for e in nd], [e.C for e in nd], s=24)
        plt.xlabel("Makespan time T")
        plt.ylabel("Total cost C (incl. penalty)")
        plt.title("Pareto set (candidate trajectories): time vs cost")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(args.out, "pareto_candidates_time_cost.png"), dpi=160)
        plt.close()

    print(f"Candidates evaluated: {len(evals)}")
    print(f"Pareto candidates: {len(nd)}")
    print(f"Wrote: {all_path}")
    print(f"Wrote: {nd_path}")


if __name__ == "__main__":
    main()

