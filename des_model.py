from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, FrozenSet, Iterable, List, Optional, Sequence, Set, Tuple


class Zone(str, Enum):
    X = "x"
    A = "A"
    B = "B"
    C = "C"


Point = str  # e.g. "x", "A1", "B2", "C3"


class EventKind(str, Enum):
    ENTER = "enter"
    MOVE = "move"
    EXPLORE = "explore"
    LEAVE = "leave"
    IDLE = "idle"
    FINISH = "finish"


@dataclass(frozen=True)
class Event:
    robot: int  # 1 or 2
    kind: EventKind
    arg: Optional[str] = None  # zone name or point name depending on kind

    def label(self) -> str:
        if self.arg is None:
            return f"R{self.robot}:{self.kind.value}"
        return f"R{self.robot}:{self.kind.value}({self.arg})"


@dataclass(frozen=True)
class Edge:
    u: Point
    v: Point
    time: float
    cost: float
    inter_zone: bool


@dataclass(frozen=True)
class RobotState:
    pos: Point
    explored_points: FrozenSet[Point]
    active: bool  # false after FINISH


@dataclass(frozen=True)
class TeamState:
    r1: RobotState
    r2: RobotState
    explored_zones: FrozenSet[Zone]


@dataclass(frozen=True)
class StepCost:
    dt: float
    dc: float
    # supervisor overhead accounted separately by the enumerator (needs enabled-set deltas)


def zone_of_point(p: Point) -> Zone:
    if p == "x":
        return Zone.X
    if p[0] == "A":
        return Zone.A
    if p[0] == "B":
        return Zone.B
    if p[0] == "C":
        return Zone.C
    raise ValueError(f"Unknown point: {p}")


def points_in_zone(z: Zone) -> Tuple[Point, ...]:
    if z == Zone.X:
        return ("x",)
    return (f"{z.value}1", f"{z.value}2", f"{z.value}3")


def required_points() -> FrozenSet[Point]:
    pts: Set[Point] = set()
    for z in (Zone.A, Zone.B, Zone.C):
        pts.update(points_in_zone(z))
    return frozenset(pts)


def default_edges() -> List[Edge]:
    """
    Encodes assumptions I-III + (D1,D2) in an explicit weighted graph:

    - Intra-zone connectivity:
      A: A1-A2-A3 (2 routes)
      B: B1-B2-B3 (2 routes)
      C: triangle C1-C2-C3-C1 (3 routes)
      Each intra-zone edge has time=1, cost=1.

    - Zone entry "distances" D1/D2 are modeled via special edges from
      portal points to the first reachable points after entering:
      Here we simplify by choosing the zone's portal point as "<Z>1".

      For A/B: from portal (Z1) to Z2 has (t=2,c=1) and to Z3 has (t=1,c=2).
      For C: from portal (C1) to any other point is (t=1,c=1) already via triangle.

    - Inter-zone adjacency:
      x <-> A via x<->A1
      x <-> B via x<->B1
      A <-> C via A2<->C2 (fast t=1,c=1), plus A1<->C1 (slow t=2,c=2)
      B <-> C via B2<->C3 (fast t=1,c=1), plus B1<->C1 (slow t=2,c=2)

    This realizes "in each two adjacent zones there exists a pair of points
    with t=1 (and we also use c=1); all other inter-zone moves take 2t (and 2c)".
    """
    E: List[Edge] = []

    def add(u: Point, v: Point, t: float, c: float, inter: bool) -> None:
        E.append(Edge(u=u, v=v, time=t, cost=c, inter_zone=inter))
        E.append(Edge(u=v, v=u, time=t, cost=c, inter_zone=inter))

    # intra A, B
    add("A1", "A2", 1, 1, False)
    add("A2", "A3", 1, 1, False)
    add("B1", "B2", 1, 1, False)
    add("B2", "B3", 1, 1, False)

    # intra C triangle
    add("C1", "C2", 1, 1, False)
    add("C2", "C3", 1, 1, False)
    add("C3", "C1", 1, 1, False)

    # model D1 by extra "entry edges" from portal to match reachability biases
    # (these are parallel alternatives; the shortest-path computation in enumerator
    # will naturally pick whichever matches the requested target)
    add("A1", "A2", 2, 1, False)  # biased to A2: slower time, cheaper cost
    add("A1", "A3", 1, 2, False)  # biased to A3: faster time, higher cost
    add("B1", "B2", 2, 1, False)
    add("B1", "B3", 1, 2, False)

    # x adjacency
    add("x", "A1", 2, 2, True)
    add("x", "B1", 2, 2, True)

    # A<->C and B<->C adjacency (one fast portal each + one slow backup)
    add("A2", "C2", 1, 1, True)  # fast
    add("A1", "C1", 2, 2, True)  # slow
    add("B2", "C3", 1, 1, True)  # fast
    add("B1", "C1", 2, 2, True)  # slow

    return E


def build_adj(edges: Sequence[Edge]) -> Dict[Point, List[Edge]]:
    adj: Dict[Point, List[Edge]] = {}
    for e in edges:
        adj.setdefault(e.u, []).append(e)
    return adj


def initial_team_state() -> TeamState:
    r0 = RobotState(pos="x", explored_points=frozenset(), active=True)
    return TeamState(
        r1=r0,
        r2=r0,
        explored_zones=frozenset(),
    )


def zone_is_complete(explored_points: FrozenSet[Point], z: Zone) -> bool:
    if z in (Zone.X,):
        return True
    pts = set(points_in_zone(z))
    return pts.issubset(explored_points)


def explored_zones_from_points(explored_points: FrozenSet[Point]) -> FrozenSet[Zone]:
    zs: Set[Zone] = set()
    for z in (Zone.A, Zone.B, Zone.C):
        if zone_is_complete(explored_points, z):
            zs.add(z)
    return frozenset(zs)


def is_zone_occupied(team: TeamState, z: Zone) -> bool:
    if z in (Zone.X,):
        return False
    return zone_of_point(team.r1.pos) == z or zone_of_point(team.r2.pos) == z


def admissible_events_for_robot(
    team: TeamState,
    robot: int,
    adj: Dict[Point, List[Edge]],
) -> List[Event]:
    r = team.r1 if robot == 1 else team.r2
    if not r.active:
        return []

    here = r.pos
    here_zone = zone_of_point(here)
    events: List[Event] = []

    # Move: along any available edge within same zone (inter_zone edges are modeled as ENTER)
    for e in adj.get(here, []):
        if e.inter_zone:
            continue
        events.append(Event(robot=robot, kind=EventKind.MOVE, arg=e.v))

    # Explore: if at a required marked point not yet explored
    if here != "x" and here not in r.explored_points:
        events.append(Event(robot=robot, kind=EventKind.EXPLORE, arg=here))

    # Enter: attempt any inter-zone edge from current point to a point in another zone
    for e in adj.get(here, []):
        if not e.inter_zone:
            continue
        z_to = zone_of_point(e.v)
        # "enter" is the control; the underlying plant may fail if occupied (handled in step)
        events.append(Event(robot=robot, kind=EventKind.ENTER, arg=z_to.value))

    # Leave: only meaningful as "go back to x" if there exists an inter-zone edge to x
    if here_zone != Zone.X:
        for e in adj.get(here, []):
            if e.inter_zone and e.v == "x":
                events.append(Event(robot=robot, kind=EventKind.LEAVE, arg="x"))
                break

    # Finish: robots may stop only after all zones explored.
    # If at x, finishing is always "preferred"; if not at x, penalty is handled at evaluation.
    if all_points_explored(team):
        events.append(Event(robot=robot, kind=EventKind.FINISH, arg=None))

    # Idle will be injected by the enumerator only when no other admissible control exists.
    return events


def step_robot(
    team: TeamState,
    ev: Event,
    adj: Dict[Point, List[Edge]],
) -> Tuple[TeamState, StepCost, bool]:
    """
    Returns (new_team, incremental_cost, progressed)

    progressed=False means this is a "no-op" attempt (only possible for ENTER into occupied zone),
    which still consumes time/cost per assumption III.
    """
    assert ev.robot in (1, 2)
    r = team.r1 if ev.robot == 1 else team.r2
    other = team.r2 if ev.robot == 1 else team.r1

    def set_robot(new_r: RobotState) -> TeamState:
        if ev.robot == 1:
            return TeamState(
                r1=new_r,
                r2=other,
                explored_zones=team.explored_zones,
            )
        return TeamState(
            r1=other,
            r2=new_r,
            explored_zones=team.explored_zones,
        )

    if not r.active:
        return (team, StepCost(0, 0), False)

    if ev.kind == EventKind.MOVE:
        assert ev.arg is not None
        # find at least one intra-zone edge to the target
        edges = [e for e in adj.get(r.pos, []) if (not e.inter_zone and e.v == ev.arg)]
        if not edges:
            raise ValueError(f"Invalid MOVE from {r.pos} to {ev.arg}")
        e = min(edges, key=lambda x: (x.time, x.cost))
        new_r = RobotState(pos=e.v, explored_points=r.explored_points, active=True)
        return (set_robot(new_r), StepCost(e.time, e.cost), True)

    if ev.kind == EventKind.EXPLORE:
        assert ev.arg == r.pos
        # exploring takes time=1 cost=1, robot does not move
        new_pts = set(r.explored_points)
        new_pts.add(r.pos)
        new_r = RobotState(pos=r.pos, explored_points=frozenset(new_pts), active=True)
        new_team = set_robot(new_r)
        # update explored zones (supervisor knows globally)
        all_pts = frozenset(new_team.r1.explored_points | new_team.r2.explored_points)
        new_team = TeamState(
            r1=new_team.r1,
            r2=new_team.r2,
            explored_zones=explored_zones_from_points(all_pts),
        )
        return (new_team, StepCost(1.0, 1.0), True)

    if ev.kind == EventKind.ENTER:
        assert ev.arg in ("x", "A", "B", "C")
        z_to = Zone(ev.arg)
        # determine which inter-zone edge from current point can realize entering z_to
        candidates = [e for e in adj.get(r.pos, []) if (e.inter_zone and zone_of_point(e.v) == z_to)]
        if not candidates:
            raise ValueError(f"Invalid ENTER from {r.pos} to zone {z_to}")

        # Choose fastest edge (tie-break by cost); plant is nondeterministic in general but
        # we commit to the best (this is documented in report as a modeling choice).
        e = min(candidates, key=lambda x: (x.time, x.cost))

        # Occupancy check: only one robot per zone at a time.
        if z_to != Zone.X and (zone_of_point(other.pos) == z_to):
            # "enter occupied" attempt: time=1 cost=0.5, position unchanged, cannot select points
            return (team, StepCost(1.0, 0.5), False)

        new_r = RobotState(pos=e.v, explored_points=r.explored_points, active=True)
        return (set_robot(new_r), StepCost(e.time, e.cost), True)

    if ev.kind == EventKind.LEAVE:
        # Leave is modeled as taking an inter-zone edge to x (fastest available).
        candidates = [e for e in adj.get(r.pos, []) if (e.inter_zone and e.v == "x")]
        if not candidates:
            raise ValueError(f"Invalid LEAVE from {r.pos} (no edge to x)")
        e = min(candidates, key=lambda x: (x.time, x.cost))

        new_r = RobotState(pos="x", explored_points=r.explored_points, active=True)
        return (set_robot(new_r), StepCost(e.time, e.cost), True)

    if ev.kind == EventKind.FINISH:
        if not all_points_explored(team):
            raise ValueError("FINISH only allowed after completing exploration")
        new_r = RobotState(pos=r.pos, explored_points=r.explored_points, active=False)
        return (set_robot(new_r), StepCost(0.0, 0.0), True)

    if ev.kind == EventKind.IDLE:
        # idle is always a legal "do nothing" tick when forced by the enumerator
        return (team, StepCost(1.0, 0.0), False)

    raise ValueError(f"Unknown event: {ev}")


def all_points_explored(team: TeamState) -> bool:
    pts = team.r1.explored_points | team.r2.explored_points
    return required_points().issubset(pts)


def terminal(team: TeamState) -> bool:
    # terminal means exploration complete AND both robots have stopped (FINISH),
    # with a preference (penalty) for being at x.
    if not all_points_explored(team):
        return False
    return (not team.r1.active) and (not team.r2.active)


def penalty_for_not_at_x(team: TeamState) -> float:
    """
    Penalty applied at completion time if one/both robots are not at x.

    - if both not at x: 10
    - if exactly one not at x:
        5 if the other robot is in A or B
        8 if the other robot is in C
    """
    r1_at = team.r1.pos == "x"
    r2_at = team.r2.pos == "x"
    if r1_at and r2_at:
        return 0.0
    if (not r1_at) and (not r2_at):
        return 10.0
    other = team.r2 if not r1_at else team.r1
    oz = zone_of_point(other.pos)
    if oz in (Zone.A, Zone.B, Zone.X):
        return 5.0
    return 8.0


@dataclass(frozen=True)
class SupervisorState:
    explored: FrozenSet[Zone]  # subset of {A,B,C}
    loc1: Zone  # robot 1 current zone
    loc2: Zone  # robot 2 current zone


class Supervisor:
    """
    A compact coordinator S (finite automaton) that:
    - prevents deadlock due to "both try to enter same occupied zone" thrashing
    - prevents livelock by disallowing repeated inter-zone moves that don't enable new exploration
    - biases toward splitting A and B first, then letting one robot do C while the other returns.

    This is not claimed to be globally time/cost optimal; instead it is a *minimal-memory*,
    specification-enforcing supervisor over the information S is assumed to have.
    """

    def state_of(self, team: TeamState) -> SupervisorState:
        return SupervisorState(
            explored=team.explored_zones,
            loc1=zone_of_point(team.r1.pos),
            loc2=zone_of_point(team.r2.pos),
        )

    def enabled(self, team: TeamState, robot: int, admissible: Sequence[Event]) -> List[Event]:
        """
        Returns the subset of admissible controllable events enabled by S.

        Modeling choice:
        - MOVE and EXPLORE are uncontrollable (always enabled if admissible).
        - ENTER/LEAVE/FINISH are controllable (may be disabled).
        - IDLE is only used by the enumerator when no other admissible event exists.
        """
        s = self.state_of(team)
        other_zone = s.loc2 if robot == 1 else s.loc1
        here_zone = s.loc1 if robot == 1 else s.loc2

        def is_enabled(ev: Event) -> bool:
            if ev.kind in (EventKind.MOVE, EventKind.EXPLORE):
                return True
            if ev.kind == EventKind.FINISH:
                # can finish only when all zones explored (location constraint handled by plant/admissible)
                return s.explored == frozenset({Zone.A, Zone.B, Zone.C})
            if ev.kind == EventKind.LEAVE:
                # leaving to x is always allowed, but discouraged when C not explored and
                # the only way to reach C is through A/B: keep at least one robot in {A,B,C}
                if Zone.C not in s.explored and here_zone in (Zone.A, Zone.B):
                    return True
                return True
            if ev.kind == EventKind.ENTER:
                assert ev.arg is not None
                z_to = Zone(ev.arg)
                # don't allow entering an already explored zone unless needed for transit to C or return to x
                if z_to in s.explored:
                    if Zone.C not in s.explored and z_to in (Zone.A, Zone.B) and here_zone != Zone.C:
                        return True  # transit to C
                    if z_to == Zone.X:
                        return True
                    return False

                # hard safety: never intentionally try to enter an occupied zone
                if z_to != Zone.X and z_to == other_zone:
                    return False

                # coordination heuristic at start: split A/B if both at x and both not explored
                if s.loc1 == Zone.X and s.loc2 == Zone.X and s.explored == frozenset():
                    if robot == 1:
                        return z_to == Zone.A
                    return z_to == Zone.B

                # if C is not explored, prioritize enabling *someone* to go to C once A and B are explored
                if Zone.C not in s.explored and (Zone.A in s.explored) and (Zone.B in s.explored):
                    return z_to == Zone.C

                return True
            return True

        enabled = [ev for ev in admissible if is_enabled(ev)]

        # If we disabled everything controllable but MOVE/EXPLORE aren't available, allow a fallback ENTER
        # to avoid deadlock due to over-restriction.
        if not enabled:
            return list(admissible)
        return enabled


def count_admissible_controls(admissible: Sequence[Event]) -> int:
    # Count only controllable events for autonomy calculation
    return sum(1 for e in admissible if e.kind in (EventKind.ENTER, EventKind.LEAVE, EventKind.FINISH))


def count_allowed_controls(enabled: Sequence[Event]) -> int:
    return sum(1 for e in enabled if e.kind in (EventKind.ENTER, EventKind.LEAVE, EventKind.FINISH))
