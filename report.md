## 1) Environment \(Z\) and quantitative structure

### Zones and marked points
- **Zones**: \(A,B,C\) (disjoint), plus origin **`x`**
- **Marked points to be visited/explored**:
  - \(A1,A2,A3\)
  - \(B1,B2,B3\)
  - \(C1,C2,C3\)

### Routes (movement graph)
The implementation uses an explicit directed weighted graph \(G=(V,E)\) (see `des_model.py::default_edges()`):

- **Within-zone routes (time=1, cost=1)**:
  - \(A\): chain \(A1\leftrightarrow A2\leftrightarrow A3\) (2 routes)
  - \(B\): chain \(B1\leftrightarrow B2\leftrightarrow B3\) (2 routes)
  - \(C\): triangle \(C1\leftrightarrow C2\leftrightarrow C3\leftrightarrow C1\) (3 routes)

- **Adjacency between zones** (only via specific point-to-point inter-zone routes):
  - `x` adjacent to \(A\) and \(B\): `x<->A1`, `x<->B1`
  - \(C\) adjacent to \(A\) and \(B\): `A2<->C2`, `B2<->C3` (fast), plus `A1<->C1`, `B1<->C1` (slow backups)

### Encoding \(D1\)–\(D2\)
Assumptions **[D1]** and **[D2]** are represented as *alternative weighted edges* from the entry/portal point of a zone:
- Entering \(A\) or \(B\) is modeled as arriving to the portal point `A1` / `B1`.
- The “reachability bias” is encoded by adding parallel alternatives:
  - `A1->A2` has an alternative edge \((t=2,c=1)\) and `A1->A3` has an alternative edge \((t=1,c=2)\) (and same pattern for \(B\)).
  - Zone \(C\) has symmetric \((t=1,c=1)\) access to its points via the triangle.

### Exploration action
At any marked point \(Yk\), the event **`explore(Yk)`** has:
- **time** \(t=1\)
- **cost** \(c=1\)
- robot **does not move** while exploring

## 2) Robot automata \(R1, R2\): states and controls

### Robot state space
Each robot \(R_i\) is a finite-state automaton with state:
\[
q_i := (\text{pos}_i,\; \text{exploredPts}_i,\; \text{active}_i)
\]
where:
- **`pos`** is one of the points in \(V=\{x,A1..A3,B1..B3,C1..C3\}\)
- **`exploredPts`** is the subset of marked points already explored by this robot
- **`active`** indicates whether the robot is still operating (`True`) or has stopped (`False`)

The **team** completes exploration when all 9 marked points are explored by either robot.

### Discrete events / controls
Allowed event types (implemented in `des_model.py::EventKind`):
- **`move(p)`**: move along an allowed **intra-zone** route to point `p`
- **`enter(Z)`**: attempt an **inter-zone** transition into zone \(Z\) via a permitted point-to-point route
- **`leave(x)`**: return to `x` (a special `enter(x)` along an inter-zone route to `x`)
- **`explore(p)`**: explore the current marked point
- **`idle`**: used only if no other control is admissible (forced waiting)
- **`finish`**: stop (allowed only after all points are explored; can stop at any location)

### Forbidden combinations (safety constraints)
These are global constraints enforced in the plant composition (`des_model.py::step_robot`):

| Forbidden situation | Meaning / rule |
|---|---|
| **Zone collision** | Robots cannot be in the same zone \(A\) or \(B\) or \(C\) at the same time (“only one robot per zone”). |
| **Opposite traversal on same inter-zone route (simultaneous)** | The pair of concurrent events \((R1:U\to V,\; R2:V\to U)\) is forbidden in the same time instant (“two robots cannot take the same route between zones in opposite directions”). |
| **Exploring `x`** | `x` is not a marked point, so it cannot be explored. |
| **Stopping early** | `finish` is forbidden until all 9 marked points are explored. |

### “Enter occupied zone” behavior
If `enter(Z)` is chosen while the other robot is currently in zone \(Z\), then:
- time \(t=1\), cost \(c=0.5\)
- **position does not change**
- no exploration starts (this is modeled as a no-op transition that still consumes time/cost)

## 3) DES network (parallel composition)

The overall plant is a network:
\[
G := R1 \parallel R2 \parallel Env \parallel Occ \parallel Dir
\]
where:
- `Env` is the weighted route graph,
- `Occ` encodes “one robot per zone”,
- `Dir` encodes the “no opposite inter-zone traversal” constraint.

In code, this network is represented by the joint state `TeamState` plus the transition function `step_robot(...)`.

## 4) Supervisor \(S\): minimal-memory coordinator

### Supervisor state
The supervisor is modeled as a finite automaton with state:
\[
s := (\text{ExploredZones},\;\text{zone}(R1),\;\text{zone}(R2))
\]
implemented as `SupervisorState` (see `des_model.py`).

This matches the assumptions:
- \(S\) knows which zones are already explored
- robots only sense whether a target zone is currently occupied when attempting to enter

### Controllable vs uncontrollable events
The supervisor **only restricts controllable events**:
- **controllable**: `enter`, `leave`, `finish`
- **uncontrollable**: `move`, `explore`

### Enabling policy (high-level)
The supervisor `Supervisor.enabled(...)` enforces:
- **no intentional “enter occupied zone”** (prevents deadlock-like thrashing)
- **progress** toward unexplored zones (prevents livelock by disallowing entering explored zones unless needed for transit to \(C\) or returning)
- **initial split**: from `x`, send one robot to \(A\) and the other to \(B\) (reduces redundant visits/idle)
- once \(A\) and \(B\) are complete, **prioritize entering \(C\)**

### Supervisor quantitative criteria
The enumerator computes:
- **Autonomy reduction** \(a=\sum a_i\) using
  \[
  a_i(q_i)=1-\frac{\#\text{allowed controllable}}{\#\text{admissible controllable}}
  \]
  summed over decision epochs.
- **Switching overhead**: each change of enabled controllable set adds **time 0.2** and **cost 0.1** per switched control (symmetric difference count).

## 5) Nondominated trajectories and Pareto visualization

### What is enumerated
`enumerate_pareto.py` performs a timed concurrent exploration of the DES:
- robots can act in parallel (each action makes the robot “busy” for its duration)
- global time is advanced to the next completion event
- admissible actions are filtered by supervisor \(S\)

Terminal trajectories end when:
- all 9 marked points are explored, and
- both robots have executed `finish` (possibly not at `x`)

### Metrics reported per trajectory
Each terminal trajectory is evaluated by:
- **makespan time** \(T\)
- **total cost** \(C\) (robots + supervisor switching + final penalty)
- **individual** \(T_1,T_2,C_1,C_2\)
- **autonomy reduction** \(a\)
- **number of switches**
- **final-state penalty** (included in \(C\)) according to:
  - 0 if both at `x`
  - 10 if neither at `x`
  - 5 if exactly one not at `x` and the other is in \(A\) or \(B\)
  - 8 if exactly one not at `x` and the other is in \(C\)

### Outputs
Running the script:

```bash
python3 enumerate_pareto.py --max-nodes 200000
```

produces:
- `out/trajectories.csv`: all terminal trajectories found under the node limit
- `out/pareto.csv`: nondominated subset in the \((T,C)\) plane
- `out/pareto_time_cost.png`: Pareto scatter (time vs cost)
- `out/pareto_time_sum.png`: makespan vs \(T_1+T_2\)

### Practical Pareto set (systematic candidate trajectories)
For coursework-scale reporting (and to avoid the full interleaving explosion), the repository also includes
`candidate_pareto.py`, which **systematically enumerates**:
- both initial splits \(R1\to A,R2\to B\) and \(R1\to B,R2\to A\),
- all \(3!\) point-visit orders inside \(A\) and \(B\),
- two choices of which robot explores \(C\) (via the fast portals),
then evaluates the timed concurrent execution and extracts the **Pareto-nondominated** subset.

Run:

```bash
python3 candidate_pareto.py
```

Outputs:
- `out/candidates.csv` (all candidates with metrics and final states)
- `out/pareto_candidates.csv` (nondominated subset in \((T,C)\))
- `out/pareto_candidates_time_cost.png`

## 6) Reference trajectory and compromise selection

To choose a compromise solution among the nondominated set, use a **reference point** \(r\) in criteria space and pick the Pareto point minimizing a distance (e.g., normalized \(L_2\)):
\[
\min_{(T,C)\in \mathcal{P}} \left\|\left(\frac{T-T^\*}{\Delta T},\frac{C-C^\*}{\Delta C}\right)\right\|_2
\]
where \((T^\*,C^\*)\) is the reference (aspiration) and \(\Delta T,\Delta C\) are scaling ranges taken from the Pareto set.

In practice:
- set \(T^\*\) to the smallest Pareto makespan,
- set \(C^\*\) to the smallest Pareto cost,
- then select the closest Pareto point as the compromise.

This selection can be added as a post-processing step if needed.

## 7) Computed solution (candidate Pareto set)

### How the computed set was generated
The computed trajectories come from `candidate_pareto.py`, which enumerates a structured family of cooperative behaviors:
- **split** the team so robots explore \(A\) and \(B\) in parallel (both assignments \(R1\to A,R2\to B\) and \(R1\to B,R2\to A\)),
- enumerate **all \(3!\)** visit orders inside \(A\) and \(B\),
- then let **exactly one robot** enter and explore \(C\) (using the fast portal),
- finally both robots execute `finish` (possibly not at `x`, which triggers the penalty).

This yields **144** evaluated candidate trajectories and a **Pareto-nondominated** subset in the \((T,C)\) plane (time vs total cost).

### Nondominated trajectories (Pareto in \((T,C)\))
From `out/pareto_candidates.csv`, the nondominated set contains 4 trajectories. In this dataset the Pareto points **coincide**:
- **Makespan time** \(T=13.0\)
- **Total cost** \(C=31.0\)

The reason the total cost is high is the **final-state penalty**: in all 4 nondominated candidates, both robots stop outside `x`, so the penalty is **10**, and \(C=(C1+C2)+10\).

| # | \(T\) | \(C\) | \(T_1\) | \(T_2\) | \(C_1\) | \(C_2\) | final (R1,R2) | High-level pattern |
|---:|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 13.0 | 31.0 | 7.0 | 13.0 | 7.0 | 14.0 | (A3, C1) | \(R1\) explores \(A\), \(R2\) explores \(B\) then \(C\) |
| 2 | 13.0 | 31.0 | 13.0 | 7.0 | 14.0 | 7.0 | (C1, B3) | \(R1\) explores \(A\) then \(C\), \(R2\) explores \(B\) |
| 3 | 13.0 | 31.0 | 13.0 | 7.0 | 14.0 | 7.0 | (C1, A3) | \(R1\) explores \(B\) then \(C\), \(R2\) explores \(A\) |
| 4 | 13.0 | 31.0 | 7.0 | 13.0 | 7.0 | 14.0 | (B3, C1) | \(R1\) explores \(B\), \(R2\) explores \(A\) then \(C\) |

The full discrete event sequences for each case are stored in `out/pareto_candidates.csv` (column `trajectory`).

### Pareto visualization
The Pareto scatter plot is saved as:
- `out/pareto_candidates_time_cost.png`

In this run, all nondominated trajectories map to the same \((T,C)\) point \((13,31)\), so the plot shows a single point.

### Selected compromise / reference trajectory
Because all nondominated points have identical \((T,C)\), the compromise choice is based on secondary preferences:
- **minimize redundancy & idling**: prefer the initial **split** \(A\) vs \(B\) (both robots productive early),
- **balance individual effort**: choose the variant where the “\(C\)-robot” also finishes the larger share of work (here \(T_2=13\) and \(C_2=14\) vs \(T_1=7\), \(C_1=7\) is acceptable if “one robot acts as the \(C\) specialist”).

Reference trajectory (chosen): **Case #1** ( \(R1:A\), \(R2:B\to C\) ) from the table above.
