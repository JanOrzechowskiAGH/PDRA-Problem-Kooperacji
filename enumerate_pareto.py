from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass, field
from heapq import heappop, heappush
from typing import Dict, FrozenSet, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt

from des_model import (
    Event,
    EventKind,
    Supervisor,
    TeamState,
    admissible_events_for_robot,
    build_adj,
    count_admissible_controls,
    count_allowed_controls,
    default_edges,
    initial_team_state,
    zone_of_point,
    penalty_for_not_at_x,
    step_robot,
    terminal,
)


@dataclass(frozen=True)
class TimedState:
    team: TeamState
    r1_rem: float
    r2_rem: float
    # last enabled *controllable* set per robot for supervisor switching overhead
    last_ctrl_enabled_1: FrozenSet[str]
    last_ctrl_enabled_2: FrozenSet[str]


@dataclass(frozen=True)
class Metrics:
    # global time (makespan)
    T: float
    # total cost: robot action costs + supervisor switching costs + penalty
    C: float
    # individual robot times and costs
    T1: float
    T2: float
    C1: float
    C2: float
    # supervisor criteria
    autonomy_reduction: float
    switches: int


@dataclass
class Path:
    evs: List[str] = field(default_factory=list)


def ctrl_signature(enabled: Sequence[Event]) -> FrozenSet[str]:
    sig = set()
    for e in enabled:
        if e.kind not in (EventKind.ENTER, EventKind.LEAVE, EventKind.FINISH):
            continue
        if e.kind == EventKind.ENTER and e.arg is not None:
            sig.add(f"ENTER:{e.arg}")
        elif e.kind == EventKind.LEAVE:
            sig.add("LEAVE:x")
        elif e.kind == EventKind.FINISH:
            sig.add("FINISH")
    return frozenset(sig)


def dominates(a: Metrics, b: Metrics, keys: Tuple[str, ...]) -> bool:
    """
    a dominates b if a is no worse on all keys and strictly better on at least one.
    """
    le_all = True
    lt_any = False
    for k in keys:
        va = getattr(a, k)
        vb = getattr(b, k)
        if va > vb + 1e-9:
            le_all = False
            break
        if va < vb - 1e-9:
            lt_any = True
    return le_all and lt_any


def pareto_filter(items: List[Tuple[Metrics, Path]], keys: Tuple[str, ...]) -> List[Tuple[Metrics, Path]]:
    nd: List[Tuple[Metrics, Path]] = []
    for m, p in items:
        dominated = False
        for m2, _ in items:
            if m2 is m:
                continue
            if dominates(m2, m, keys):
                dominated = True
                break
        if not dominated:
            nd.append((m, p))
    return nd


def add_switch_overhead(
    m: Metrics,
    prev: FrozenSet[str],
    now: FrozenSet[str],
) -> Tuple[Metrics, int]:
    delta = len(prev.symmetric_difference(now))
    if delta == 0:
        return m, 0
    # one switch: cost 0.1 and time 0.2
    add_c = 0.1 * delta
    add_t = 0.2 * delta
    return (
        Metrics(
            T=m.T + add_t,
            C=m.C + add_c,
            T1=m.T1,
            T2=m.T2,
            C1=m.C1,
            C2=m.C2,
            autonomy_reduction=m.autonomy_reduction,
            switches=m.switches + delta,
        ),
        delta,
    )


def expand_joint_actions(
    team: TeamState,
    r1_free: bool,
    r2_free: bool,
    adj,
    sup: Supervisor,
) -> List[Tuple[Optional[Event], Optional[Event], FrozenSet[str], FrozenSet[str], float]]:
    """
    Returns joint actions (ev1, ev2, enabled_sig1, enabled_sig2, autonomy_increment).

    autonomy_increment is the sum of autonomy reductions for robots that are free at this epoch.
    """
    admiss1 = admissible_events_for_robot(team, 1, adj) if r1_free else []
    admiss2 = admissible_events_for_robot(team, 2, adj) if r2_free else []
    en1 = sup.enabled(team, 1, admiss1) if r1_free else []
    en2 = sup.enabled(team, 2, admiss2) if r2_free else []

    # If a robot is free and has no enabled events, it must idle (time=1, cost=0)
    if r1_free and not en1:
        en1 = [Event(robot=1, kind=EventKind.IDLE, arg=None)]
        admiss1 = en1
    if r2_free and not en2:
        en2 = [Event(robot=2, kind=EventKind.IDLE, arg=None)]
        admiss2 = en2

    sig1 = ctrl_signature(en1) if r1_free else frozenset()
    sig2 = ctrl_signature(en2) if r2_free else frozenset()

    def autonomy_for(adm: Sequence[Event], en: Sequence[Event]) -> float:
        adm_c = count_admissible_controls(adm)
        if adm_c == 0:
            return 0.0
        allow_c = count_allowed_controls(en)
        return 1.0 - (allow_c / adm_c)

    actions: List[Tuple[Optional[Event], Optional[Event], FrozenSet[str], FrozenSet[str], float]] = []

    # Asynchronous DES: when both robots are free, events need not occur simultaneously.
    # We model this by allowing a joint step where only one robot fires an event
    # (the other chooses None, meaning "no event at this instant").
    choices1: List[Optional[Event]] = [None] + (list(en1) if r1_free else [])
    choices2: List[Optional[Event]] = [None] + (list(en2) if r2_free else [])

    for e1 in (choices1 if r1_free else [None]):
        for e2 in (choices2 if r2_free else [None]):
            if e1 is None and e2 is None:
                continue
            a_inc = 0.0
            if r1_free and e1 is not None:
                a_inc += autonomy_for(admiss1, en1)
            if r2_free and e2 is not None:
                a_inc += autonomy_for(admiss2, en2)
            actions.append((e1, e2, sig1, sig2, a_inc))

    return actions


def apply_joint(
    team: TeamState,
    e1: Optional[Event],
    e2: Optional[Event],
    adj,
) -> Tuple[TeamState, float, float, float, float, float, float, List[str]]:
    """
    Apply events (if any) "simultaneously".

    Returns:
      (new_team, r1_dt, r2_dt, r1_dc, r2_dc, team_dt_max, team_dc_sum, labels)

    team_dt_max is not used directly; we treat time as "robot busy time" and the global
    clock advances by min busy times, but we still need per-robot remaining times.
    """
    labels: List[str] = []
    r1_dt = r2_dt = 0.0
    r1_dc = r2_dc = 0.0

    cur = team
    # Reject obvious concurrent conflicts:
    # - zone exclusivity: both entering same non-x zone
    # - no opposite-direction use of the same inter-zone route (approximated at zone level)
    if e1 is not None and e2 is not None:
        if e1.kind == EventKind.ENTER and e2.kind == EventKind.ENTER and e1.arg == e2.arg and e1.arg != "x":
            raise ValueError("Both robots cannot enter the same zone simultaneously")
        def inter_dir(ev: Event, pos: str) -> Optional[Tuple[str, str]]:
            if ev.kind == EventKind.ENTER and ev.arg is not None:
                return (zone_of_point(pos).value, ev.arg)
            if ev.kind == EventKind.LEAVE:
                return (zone_of_point(pos).value, "x")
            return None

        d1 = inter_dir(e1, team.r1.pos)
        d2 = inter_dir(e2, team.r2.pos)
        if d1 is not None and d2 is not None:
            if d1[0] == d2[1] and d1[1] == d2[0]:
                raise ValueError("Forbidden opposite-direction concurrent inter-zone traversal")

    if e1 is not None:
        cur, sc, _ = step_robot(cur, e1, adj)
        r1_dt += sc.dt
        r1_dc += sc.dc
        labels.append(e1.label())
    if e2 is not None:
        cur, sc, _ = step_robot(cur, e2, adj)
        r2_dt += sc.dt
        r2_dc += sc.dc
        labels.append(e2.label())

    return cur, r1_dt, r2_dt, r1_dc, r2_dc, max(r1_dt, r2_dt), (r1_dc + r2_dc), labels


def state_key(ts: TimedState) -> Tuple:
    # hashable key without embedding large explored-point sets twice
    return (
        ts.team.r1.pos,
        tuple(sorted(ts.team.r1.explored_points)),
        ts.team.r1.active,
        ts.team.r2.pos,
        tuple(sorted(ts.team.r2.explored_points)),
        ts.team.r2.active,
        tuple(sorted(z.value for z in ts.team.explored_zones)),
        round(ts.r1_rem, 6),
        round(ts.r2_rem, 6),
        tuple(sorted(ts.last_ctrl_enabled_1)),
        tuple(sorted(ts.last_ctrl_enabled_2)),
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-nodes", type=int, default=200000)
    ap.add_argument("--out", type=str, default="out")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    edges = default_edges()
    adj = build_adj(edges)
    sup = Supervisor()

    start = TimedState(
        team=initial_team_state(),
        r1_rem=0.0,
        r2_rem=0.0,
        last_ctrl_enabled_1=frozenset(),
        last_ctrl_enabled_2=frozenset(),
    )

    start_m = Metrics(T=0.0, C=0.0, T1=0.0, T2=0.0, C1=0.0, C2=0.0, autonomy_reduction=0.0, switches=0)
    start_p = Path(evs=[])

    # Priority queue by global time, then cost (helps find short solutions early)
    pq: List[Tuple[float, float, int, TimedState, Metrics, Path]] = []
    seq = 0
    heappush(pq, (0.0, 0.0, seq, start, start_m, start_p))

    # For pruning: per state key, keep a list of nondominated metric summaries on (T,C)
    seen: Dict[Tuple, List[Metrics]] = {}

    terminals: List[Tuple[Metrics, Path]] = []
    expanded = 0

    while pq and expanded < args.max_nodes:
        _, _, _, ts, m, p = heappop(pq)
        expanded += 1

        if terminal(ts.team):
            # terminal cost includes penalty for not returning to x
            pen = penalty_for_not_at_x(ts.team)
            m_term = Metrics(
                T=m.T,
                C=m.C + pen,
                T1=m.T1,
                T2=m.T2,
                C1=m.C1 + (pen if (not ts.team.r1.active and ts.team.r1.pos != "x" and ts.team.r2.pos == "x") else 0.0),
                C2=m.C2 + (pen if (not ts.team.r2.active and ts.team.r2.pos != "x" and ts.team.r1.pos == "x") else 0.0),
                autonomy_reduction=m.autonomy_reduction,
                switches=m.switches,
            )
            terminals.append((m_term, p))
            continue

        # Deterministic time advance when no robot can act right now.
        # This covers:
        # - both robots busy
        # - one robot busy while the other has already finished
        r1_can_act = ts.r1_rem <= 1e-9 and ts.team.r1.active
        r2_can_act = ts.r2_rem <= 1e-9 and ts.team.r2.active
        if (not r1_can_act) and (not r2_can_act) and (ts.r1_rem > 1e-9 or ts.r2_rem > 1e-9):
            dt = min([x for x in (ts.r1_rem, ts.r2_rem) if x > 1e-9])
            new_ts = TimedState(
                team=ts.team,
                r1_rem=max(0.0, ts.r1_rem - dt),
                r2_rem=max(0.0, ts.r2_rem - dt),
                last_ctrl_enabled_1=ts.last_ctrl_enabled_1,
                last_ctrl_enabled_2=ts.last_ctrl_enabled_2,
            )
            new_m = Metrics(
                T=m.T + dt,
                C=m.C,
                T1=m.T1,
                T2=m.T2,
                C1=m.C1,
                C2=m.C2,
                autonomy_reduction=m.autonomy_reduction,
                switches=m.switches,
            )
            seq += 1
            heappush(pq, (new_m.T, new_m.C, seq, new_ts, new_m, p))
            continue

        # Decision epoch: at least one robot is free.
        r1_free = r1_can_act
        r2_free = r2_can_act

        # if a robot already finished, it has no decisions
        actions = expand_joint_actions(ts.team, r1_free, r2_free, adj, sup)

        for e1, e2, sig1, sig2, a_inc in actions:
            try:
                new_team, r1_dt, r2_dt, r1_dc, r2_dc, _, _, labels = apply_joint(ts.team, e1, e2, adj)
            except ValueError:
                continue

            new_ts = TimedState(
                team=new_team,
                r1_rem=(r1_dt if r1_free else ts.r1_rem),
                r2_rem=(r2_dt if r2_free else ts.r2_rem),
                last_ctrl_enabled_1=(sig1 if (r1_free and e1 is not None) else ts.last_ctrl_enabled_1),
                last_ctrl_enabled_2=(sig2 if (r2_free and e2 is not None) else ts.last_ctrl_enabled_2),
            )

            new_m = Metrics(
                T=m.T,
                C=m.C + r1_dc + r2_dc,
                T1=m.T1 + (r1_dt if r1_free else 0.0),
                T2=m.T2 + (r2_dt if r2_free else 0.0),
                C1=m.C1 + r1_dc,
                C2=m.C2 + r2_dc,
                autonomy_reduction=m.autonomy_reduction + a_inc,
                switches=m.switches,
            )

            # supervisor switching overhead: charge only when that robot actually receives a control now
            if r1_free and e1 is not None:
                new_m, _ = add_switch_overhead(new_m, ts.last_ctrl_enabled_1, sig1)
            if r2_free and e2 is not None:
                new_m, _ = add_switch_overhead(new_m, ts.last_ctrl_enabled_2, sig2)

            new_p = Path(evs=p.evs + labels)

            k = state_key(new_ts)
            # prune by (T,C) dominance inside the same state
            bucket = seen.setdefault(k, [])
            dominated = False
            for old in bucket:
                if (old.T <= new_m.T + 1e-9) and (old.C <= new_m.C + 1e-9):
                    dominated = True
                    break
            if dominated:
                continue
            # remove metrics dominated by the new one
            bucket[:] = [old for old in bucket if not ((new_m.T <= old.T + 1e-9) and (new_m.C <= old.C + 1e-9))]
            bucket.append(new_m)

            seq += 1
            heappush(pq, (new_m.T, new_m.C, seq, new_ts, new_m, new_p))

    # Pareto analysis on terminal trajectories
    keys_tc = ("T", "C")
    pareto_tc = pareto_filter(terminals, keys_tc)

    all_path = os.path.join(args.out, "trajectories.csv")
    with open(all_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["T", "C", "T1", "T2", "C1", "C2", "autonomy_reduction", "switches", "trajectory"])
        for m0, p0 in terminals:
            w.writerow([m0.T, m0.C, m0.T1, m0.T2, m0.C1, m0.C2, m0.autonomy_reduction, m0.switches, " | ".join(p0.evs)])

    pareto_path = os.path.join(args.out, "pareto.csv")
    with open(pareto_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["T", "C", "T1", "T2", "C1", "C2", "autonomy_reduction", "switches", "trajectory"])
        for m0, p0 in sorted(pareto_tc, key=lambda x: (x[0].T, x[0].C)):
            w.writerow([m0.T, m0.C, m0.T1, m0.T2, m0.C1, m0.C2, m0.autonomy_reduction, m0.switches, " | ".join(p0.evs)])

    # plots
    if pareto_tc:
        xs = [m0.T for m0, _ in pareto_tc]
        ys = [m0.C for m0, _ in pareto_tc]
        plt.figure(figsize=(7, 5))
        plt.scatter(xs, ys, s=24)
        plt.xlabel("Makespan time T")
        plt.ylabel("Total cost C (incl. penalty + supervisor)")
        plt.title("Pareto set: time vs cost")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(args.out, "pareto_time_cost.png"), dpi=160)
        plt.close()

        xs2 = [m0.T for m0, _ in pareto_tc]
        ys2 = [m0.T1 + m0.T2 for m0, _ in pareto_tc]
        plt.figure(figsize=(7, 5))
        plt.scatter(xs2, ys2, s=24)
        plt.xlabel("Makespan time T")
        plt.ylabel("Sum of robot times T1+T2")
        plt.title("Pareto set: makespan vs total robot time")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(args.out, "pareto_time_sum.png"), dpi=160)
        plt.close()

    print(f"Expanded nodes: {expanded}")
    print(f"Terminal trajectories: {len(terminals)}")
    print(f"Pareto(T,C) trajectories: {len(pareto_tc)}")
    print(f"Wrote: {all_path}")
    print(f"Wrote: {pareto_path}")


if __name__ == "__main__":
    main()

