# Introduction

Online complete coverage requires a robot to cover all accessible free space in a workspace, while avoiding obstacles. In the online setting, the robot does not have a complete prior map and must instead build knowledge during motion.

This report focuses on BA*, an online complete coverage algorithm that combines boustrophedon motion for local sweeping with A* on a discrete tiling model for intelligent backtracking. The key idea is that the robot sweeps efficiently most of the time, but when it reaches a dead-end or boundary situation, it selects a new starting point and returns to it through a collision-free path computed using A* on tiling with a smoothing post-process (A*SPT).

The goal of this report is to describe the BA* method as implemented in our simulation, and to prepare an evaluation structure covering coverage rate, path length, and number of boustrophedon regions.

# Background and Related Work

## Boustrophedon Cellular Decomposition as an offline baseline

A common baseline for coverage planning in known environments is Boustrophedon Cellular Decomposition (BCD). BCD decomposes the free space into cells using a sweep-line process; it then builds an adjacency graph of cells and plans a traversal to cover each cell with boustrophedon-like motions. BCD is effective when the map is known, but it assumes the workspace model is available before execution.

BA* is related in spirit because it uses boustrophedon-style sweeping to achieve efficient coverage, but BA* operates online and does not rely on a precomputed decomposition.

## A* as used inside BA*

BA* uses A* as a local planner on a discrete tiling graph, not as a general continuous-space motion planner. The algorithm maintains a tiling model of discovered space and constructs a graph whose nodes are tiles and whose edges connect neighboring tiles. When BA* needs to backtrack from a current critical point to a selected starting point, it runs A* on this graph.

In our use, A* employs:

* A cost-to-come term $g(s)$ that accumulates path cost on the tiling graph.
* A heuristic $h(s)$ that is the straight-line distance from a tile to the goal tile center.

Under standard A* assumptions (non-negative edge weights and an admissible heuristic), A* finds a shortest path on the tiling graph. However, the resulting path may contain unnecessary turns on a grid-like graph; this motivates a smoothing step (A*SPT) after A*.


# BA* Method

This section presents the BA* algorithm as used in our implementation. The method consists of the workspace and robot model, the tiling model, the boustrophedon coverage mode, and the backtracking mechanism.

To illustrate every step of the algorithm, we will track a single scenario:

* **Unified Scenario:** A robot named "Bot" with radius $r=5$ (diameter $10$) operates in a room.
* **Phase 1 (Start):** Bot starts at $(10, 10)$ facing East.
* **Phase 2 (Stuck):** Later, Bot ends up at $(50, 50)$ and gets stuck (Critical Point).
* **Phase 3 (Recovery):** Bot calculates a plan to return to a safe spot at $(40, 50)$.

---

## Workspace and robot model

BA* targets online complete coverage in an unknown closed workspace. The robot is assumed to move in planar space with configuration:
$$
q = [x, y, \theta]^T,
$$
where $(x,y)\in\mathbb{R}^2$ is the robot center position in the plane and $\theta$ is the heading angle. The robot is modeled as a circle with radius $r$, and the free configuration space is denoted $C_{\text{free}}$.

Motion primitives used in the report are:

* **Forward translation** of distance $d$ at heading $\theta$:
    $$
    q'=
    \begin{bmatrix}
    x' \\
    y' \\
    \theta'
    \end{bmatrix}
    =
    \begin{bmatrix}
    x \\
    y \\
    \theta
    \end{bmatrix}
    +
    \begin{bmatrix}
    d\cos\theta \\
    d\sin\theta \\
    0
    \end{bmatrix}.
    $$

* **In-place rotation** by angle $\alpha$:
    $$
    q'=
    \begin{bmatrix}
    x' \\
    y' \\
    \theta'
    \end{bmatrix}
    =
    \begin{bmatrix}
    x \\
    y \\
    \theta
    \end{bmatrix}
    +
    \begin{bmatrix}
    0 \\
    0 \\
    \alpha
    \end{bmatrix}.
    $$

BA* uses a discrete heading set aligned with the four cardinal directions. In this report, we follow the convention (as stated in the draft) that $0$ points East and:

* East: $0$
* North: $\pi/2$
* West: $\pi$
* South: $-\pi/2$

### Unified Example: Phase 1 (Movement)

**Scenario:** Our robot starts at $(10,10)$ facing East. It rotates North and moves forward one step.

1.  **Initial State:**
    $$q_{\text{start}}=\begin{bmatrix}10 \\ 10 \\ 0\end{bmatrix}.$$

2.  **Rotation:** Rotate to face North ($\alpha=\pi/2$).
    $$
    q_{\text{rotated}}=
    \begin{bmatrix}
    10 \\
    10 \\
    0
    \end{bmatrix}
    +
    \begin{bmatrix}
    0 \\
    0 \\
    \pi/2
    \end{bmatrix}
    =
    \begin{bmatrix}
    10 \\
    10 \\
    \pi/2
    \end{bmatrix}.
    $$

3.  **Translation:** Move forward by radius distance $d=5$.
    $$
    q_{\text{final}}=
    \begin{bmatrix}
    10 \\
    10 \\
    \pi/2
    \end{bmatrix}
    +
    \begin{bmatrix}
    5\cos(\pi/2) \\
    5\sin(\pi/2) \\
    0
    \end{bmatrix}
    =
    \begin{bmatrix}
    10 \\
    15 \\
    \pi/2
    \end{bmatrix}.
    $$

---

## Discrete tiling model $M$

### Tile definition and incremental construction

BA* constructs a tiling model incrementally. Each visited configuration induces a tile index in a regular grid, where the tile spacing depends on the robot radius. In our report, a tile state is denoted:
$$
s=(x,y,2r)
$$
where the parameters are defined as follows:

* **$(x, y)$:** This is the center point of the tile. It corresponds to the robot's position in the workspace at the moment the tile was created.
* **$2r$:** This represents the size (side length) of the square tile. Since $r$ is the robot's radius, $2r$ corresponds to the robot's diameter.
* **Area:** Consequently, the area covered by a single tile is the square of the diameter, $4r^2$. The tile physically represents the square bounding box that fully contains the circular robot at that position.

The paper defines the tiling model as a set of tiles generated in free space:
$$
M = \{s \mid s \in C_{\text{free}}\}
$$

* **$C_{\text{free}}$:** This represents the "Free Space" (Configuration Space), which is the set of all valid configurations where the robot does not intersect with obstacles.
* **$M$:** This is the robot's accumulated memory or map. It starts as an empty set and grows incrementally as the robot moves and "stamps" new tiles onto the grid.

#### Explanation of $C_{free}$

In the BA* algorithm, the robot's environment is divided into "valid" and "invalid" states. The Free Space ($C_{\text{free}}$) represents the set of all "valid" configurations—meaning every possible position and orientation where the robot can physically exist without crashing into a wall or object.

The mathematical definition is:
$$
C_{\text{free}}=\left\{ q\in C \;\middle|\; A(q)\cap\left(\bigcup_i O_i\right)=\varnothing \right\}
$$

Where:
* **$q$ (Configuration):** The specific state of the robot at a single moment (defined earlier as $[x, y, \theta]^T$).
* **$C$ (Configuration Space):** The "universe" of all possible configurations, regardless of obstacles.
* **$A(q)$ (Robot Footprint):** The physical portion of the workspace occupied by the robot $A$ when placed at configuration $q$.
* **$O_i$ (Obstacle $i$):** A single obstacle in the workspace.
* **$\bigcup_i O_i$ (Obstacle Region):** The union of all obstacles, representing all solid, non-traversable points.
* **$\cap$ (Intersection) & $\varnothing$ (Empty Set):** The intersection checks for overlap. The formula requires this overlap to be the empty set (zero overlap).

> **Semantic translation:** "The Free Space ($C_{\text{free}}$) consists of all configurations ($q$) where the robot's physical body ($A(q)$) does not touch any part of the combined obstacle region."

### Covered vs visited and a consistent discovered state

To make predicates like “blocked”, “covered”, and “uncovered” explicit, we maintain a discovered tiling state as a partial function:
$$
\hat{M}:\mathbb{Z}^2 \to \{\text{unknown},\text{covered},\text{obstacle}\}.
$$

From $\hat{M}$ we derive:
$$
C_{\text{cov}}=\{s: \hat{M}(s)=\text{covered}\},\quad
C_{\text{obs}}=\{s: \hat{M}(s)=\text{obstacle}\},\quad
C_{\text{unk}}=\{s: \hat{M}(s)=\text{unknown}\}.
$$

In this report, the “current tiling model” used by BA* for planning is the covered set:
$$
M \equiv C_{\text{cov}}.
$$

We use the convention from the draft that a **blocked position** is “either an obstacle or a covered tile”:
$$
\text{isBlocked}(s)\iff \hat{M}(s)\in\{\text{covered},\text{obstacle}\},
\qquad
\text{isUncovered}(s)\iff \hat{M}(s)=\text{unknown}.
$$

The key invariant emphasized in this report is:
* **Covered at most once:** a tile becomes covered when first swept by boustrophedon motion and is never “uncovered” again.

### Unified Example: Phase 1 (Tiling)

**Scenario:** Continuing from Phase 1, the robot creates tiles as it moves.

1.  **Tile 1:** When at $(10, 10)$, it stamped:
    $$s_1 = (10, 10, 10).$$
    *(Center 10,10; Size 10)*
2.  **Tile 2:** After moving to $(10, 15)$, it stamps:
    $$s_2 = (10, 15, 10).$$
3.  **Update Map:**
    $$M = \{ (10, 10, 10), (10, 15, 10) \}.$$
    The map $M$ now contains these two discrete squares.

---

## Boustrophedon motion as coverage mode

### Boustrophedon Motion and Critical Points

In BA*, **boustrophedon motion (BM)** is the default coverage behavior. The robot moves in parallel sweep lines across free space, alternating direction at boundaries, and marking tiles as covered as it sweeps.

The coverage process continues until BM can no longer extend the sweep without hitting blocked tiles or obstacles, at which point BA* declares a critical point and switches to backtracking.

### Critical point definition

A tile $s$ is treated as a **critical point** when it is locally enclosed by blocked tiles in the 4-neighborhood. The formal definition is:
$$
s \text{ is critical } \iff \forall s'\in N_4(s),\ \text{isBlocked}(s').
$$

Where:

* **$s$ (Current Tile):** The tile the robot occupies.
* **$N_4(s)$ (4-Neighborhood):** The set of four immediate neighbors: North, South, East, West.
* **$\text{isBlocked}(s')$:** Returns True if neighbor $s'$ is an obstacle or already covered.

> **Semantic translation:** "Tile $s$ is critical **if and only if** **all** four neighbors are **blocked**."

### Unified Example: Phase 2 (Stuck)

**Scenario:** Time has passed. The robot is now deep in the room at **$(50, 50)$**. It needs to check if it is stuck.

1.  **Current Tile:** $s = (50, 50, 10)$.
2.  **Neighbors ($N_4$):**
    * **North $(50, 60)$:** Sensor sees a wall $\rightarrow$ **Blocked**.
    * **South $(50, 40)$:** Map says visited $\rightarrow$ **Blocked**.
    * **East $(60, 50)$:** Map says visited $\rightarrow$ **Blocked**.
    * **West $(40, 50)$:** Map says visited $\rightarrow$ **Blocked**.
3.  **Result:** All 4 neighbors are blocked.
    $$\text{isCritical}(50, 50) = \text{True}.$$
    The robot declares a **Critical Point** and stops sweeping.

---

## Backtracking list $L$

BA* maintains a list $L$ of candidate tiles that may serve as future starting points for new boustrophedon sweeps. These are typically tiles on boundaries between covered and uncovered regions, detected using local neighbor relationships.

### Neighbor indexing
The 8-neighborhood of a tile $s$ is defined as:
$$
N(s)=\{s_1,s_2,s_3,s_4,s_5,s_6,s_7,s_8\},
$$
corresponding to **east, north-east, north, north-west, west, south-west, south, south-east**.

The 4-neighborhood is:
$$
N_8(s)=N(s),\qquad N_4(s)=\{s_1,s_3,s_5,s_7\}.
$$

* **$s$:** The central tile currently being analyzed.
* **$s_1 \dots s_8$:** The eight surrounding tiles, indexed counter-clockwise starting from East ($s_1$).

### Boundary indicator $b$
We define a function to detect "free-to-blocked" edges:
$$
b(s_i,s_j)=
\begin{cases}
1, & \text{if } s_i \text{ is free and } s_j \text{ is blocked} \\
0, & \text{otherwise}
\end{cases}
$$

Using $\hat{M}$, one concrete realization used in the draft is:
$$
s_i \text{ is free } \iff \hat{M}(s_i)=\text{unknown},\qquad
s_j \text{ is blocked } \iff \hat{M}(s_j)\in\{\text{covered},\text{obstacle}\}.
$$

Where: 

* **$b(s_i, s_j)$:** A binary check that returns **1** only if the first neighbor ($s_i$) is unexplored and the second neighbor ($s_j$) is an obstacle or already visited.
* **$\hat{M}(s_i)$:** The status of the tile in the robot's memory (Unknown, Covered, or Obstacle).

**Semantic translation:**
> "The boundary indicator $b$ for a pair of tiles is **1** if the first tile is **free** (unknown) AND the second tile is **blocked** (obstacle/covered). Otherwise, it is **0**."


### Corner detector $\mu(s)$
The draft defines a corner score $\mu(s)$ by summing boundary indicators on selected neighbor pairs:
$$
\mu(s)=b(s_1,s_8)+b(s_1,s_2)+b(s_5,s_6)+b(s_5,s_4)+b(s_7,s_6)+b(s_7,s_8).
$$

Where:

* **$\mu(s)$:** A "corner score" integer. If this is greater than 0, the tile is sitting on a geometric corner of the unexplored area.
* **The Sum:** It checks six specific neighbor pairs (like East vs South-East, or West vs South-West) to capture different corner orientations.

> **Semantic translation:** "The corner score is the sum of specific neighbor pairs that represent geometric corners. If $\mu > 0$, the tile is on a corner."

### List Construction
A tile is added to the backtracking list when it is a “corner-like” boundary point:
$$
L=\{ s \mid s\in M \text{ and } \mu(s)\ge 1 \}.
$$

### Unified Example: Phase 2 (List Check)

**Scenario:** The robot is stuck at $(50, 50)$, so it checks its memory $M$ for backtracking candidates. It analyzes the tile at **$(40, 50)$** (West of current).

1.  **Analyze Tile:** $s = (40, 50)$.
2.  **Neighbors of $(40, 50)$:**
    * $s_1$ (East): $(50, 50)$ is **Blocked** (Current pos).
    * $s_5$ (West): $(30, 50)$ is **Free** (Unknown area!).
    * $s_4$ (North-West): $(30, 60)$ is **Blocked** (Wall).
3.  **Calculate $\mu$:**
    * Check pair $(s_5, s_4)$: $s_5$ is Free, $s_4$ is Blocked.
    * $b(s_5, s_4) = 1$.
4.  **Result:** $\mu(40, 50) \ge 1$.
    The tile **$(40, 50)$** is added to list $L$.

---

## Selection of the next starting point

When BA* reaches a critical point $s_{cp}$, it selects a next starting point $s_{sp}\in L$ that minimizes a cost criterion. The report uses a high-level selection:
$$
s_{sp}=\arg\min_{s\in L} f(s,s_{cp}).
$$

Where:

* **$s_{sp}$ (Starting Point):** The "winner" tile selected from the list. This will be the start of the next boustrophedon sweep.
* **$L$ (Backtracking List):** The list of all candidate "corner" tiles identified earlier.
* **$\arg\min$:** A mathematical operator that finds the *argument* (in this case, the tile $s$) that produces the *minimum* value for the function $f$.
* **$f(s, s_{cp})$:** A generalized cost function representing the "expense" (distance, time, or energy) to travel from the current stuck point ($s_{cp}$) to a candidate tile ($s$).

**Semantic translation:**
> "The next starting point $s_{sp}$ is the tile $s$ from the list $L$ that minimizes the cost function $f$ relative to the current critical point $s_{cp}$."

**In simple terms:**
The robot looks at its "To-Do List" and picks the easiest (closest/cheapest) location to travel to right now.

---

In practice, the selection is often based on the shortest backtracking path length computed on the tiling graph:
$$
J(s_{cp},s)=\text{len}\big(\hat{P}(s_{cp}\to s)\big),
$$
$$
s_{sp}=\arg\min_{s\in L} J(s_{cp},s).
$$

Where:

* **$J(s_{cp}, s)$:** The specific cost value used in this implementation (Path Length).
* **$\hat{P}$ (Smoothed Path):** The collision-free path calculated by the path planner (A*SPT), moving from the critical point to candidate $s$.
* **$\text{len}(\dots)$:** The total physical length of that path in meters/units.

**Semantic translation:**
> "The cost $J$ is the length of the smoothed path $\hat{P}$ from the critical point to the candidate. The selected starting point is the candidate $s$ that minimizes this path length."

**In simple terms:**
The robot doesn't just measure a straight line through walls. It calculates the actual walking distance to each candidate and picks the one with the shortest walk.

---

### Unified Example: Phase 3 (Selection)

**Scenario:** The robot is currently stuck at the **Critical Point $s_{cp} = (50, 50)$**. It consults its Backtracking List $L$, which contains two valid candidates identified during previous exploration.

1.  **Candidates in $L$:**
    * **Candidate A:** Tile at **$(40, 50)$**. (This is just West of the current position).
    * **Candidate B:** Tile at **$(10, 10)$**. (This was the starting point, far away).

2.  **Calculate Path Costs ($J$):**
    * **Path to A:** The planner finds a direct path.
        $$J((50,50), (40,50)) = 10 \text{ units}.$$
    * **Path to B:** The planner finds a path traversing back across the whole room.
        $$J((50,50), (10,10)) \approx 56 \text{ units}.$$

3.  **Selection ($\arg\min$):**
    The robot compares the costs: $10 < 56$.

4.  **Result:**
    $$s_{sp} = (40, 50).$$
    The robot selects **$(40, 50)$** as its next destination.
---

## Backtracking path planning

### A* on the tiling graph and Smoothing

BA* first plans a collision-free path on the tiling graph from the critical point $s_{cp}$ to the selected starting point $s_{sp}$. Since grid-based paths can be jagged, the path $P$ is then smoothed using A*SPT (A* with Smoothed Path on Tiling) to reduce unnecessary turns.

* **A\* Search:** Finds the shortest path by hopping between adjacent tiles in the robot's memory ($M$).
* **A\*SPT:** A post-processing step that shortcuts the path. It connects the current waypoint to the **farthest** visible waypoint in the path list, skipping intermediate tiles if a straight line of sight exists.

---

### Following the backtracking path

To execute the path, the robot moves from its current pose $q$ to the center of the next waypoint tile $s_{i+1}$. The control commands are computed using three formulas:

* **Heading ($\beta$):**
    $$
    \beta=\arctan\frac{y_{i+1}-y}{x_{i+1}-x}
    $$
* **Rotation ($\alpha$):**
    $$
    \alpha=\beta-\theta
    $$
* **Distance ($d$):**
    $$
    d=\sqrt{(x_{i+1}-x)^2+(y_{i+1}-y)^2}
    $$

#### Explanation of parameters

* **$(x, y)$:** The robot's current position coordinates.
* **$(x_{i+1}, y_{i+1})$:** The target coordinates (the center of the next tile in the smoothed path).
* **$\theta$ (Current Heading):** The direction the robot is currently facing.
* **$\beta$ (Required Heading):** The absolute angle (in the workspace frame) the robot *needs* to face to point directly at the target.
* **$\alpha$ (Steering Angle):** The relative turn required. It is the difference between where the robot *should* look ($\beta$) and where it *is* looking ($\theta$).
* **$d$ (Travel Distance):** The straight-line Euclidean distance to the target.

#### Semantic translation

* **Formula 1 ($\beta$):** "Calculate the absolute direction from 'Here' to 'There' using the inverse tangent (arc-tangent) of the slope."
* **Formula 2 ($\alpha$):** "The turn amount is the Target Angle minus my Current Angle."
* **Formula 3 ($d$):** "The distance to travel is the length of the hypotenuse between the two points."

---

### Unified Example: Phase 3 (Execution)

**Scenario:** The robot is currently at the stuck point $(50, 50)$ and has selected $(40, 50)$ as its next destination. Since these tiles are adjacent and have a clear line of sight, the path planner outputs the target directly.

1.  **Current State ($q$):**
    * Position: **$(50, 50)$**
    * Heading: **$\pi/2$** (Facing North, $90^{\circ}$)

2.  **Target Waypoint ($s_{i+1}$):**
    * Coordinates: **$(40, 50)$**

3.  **Compute Control Commands:**

    * **Step A: Calculate Heading ($\beta$)**
        $$\beta = \arctan\left(\frac{50 - 50}{40 - 50}\right) = \arctan\left(\frac{0}{-10}\right)$$
        In trigonometry, coordinates $(-10, 0)$ correspond to the angle **$\pi$** radians ($180^{\circ}$ or West).
        $$\beta = \pi$$

    * **Step B: Calculate Rotation ($\alpha$)**
        $$\alpha = \beta - \theta = \pi - \frac{\pi}{2} = \frac{\pi}{2}$$
        A positive $\pi/2$ indicates a **Counter-Clockwise (Left)** turn of $90^{\circ}$.

    * **Step C: Calculate Distance ($d$)**
        $$d = \sqrt{(40 - 50)^2 + (50 - 50)^2} = \sqrt{(-10)^2 + 0} = \sqrt{100} = 10$$

4.  **Execute:**
    The robot executes a **Left Turn** ($\pi/2$) and moves forward **10 units**. It arrives at $(40, 50)$, completes the backtracking phase, and is ready to start a new coverage sweep.

# Implementation Details

This section describes how the BA* method is implemented in code, focusing on the tiling model, neighborhood queries, and the execution flow that connects boustrophedon motion (BM) with A* based backtracking.

## Data structures

### Discovered tiling state $\hat{M}$

We store the discovered map state as a mapping from **discrete tile indices** to a finite tile state:

$$
\hat{M}:\mathbb{Z}^2 \to {\text{unknown},\text{covered},\text{obstacle}}.
$$

In code, $\hat{M}$ is implemented as a grid or a hash map keyed by integer coordinates, for example `(i, j)`. Tiles not present in the map are treated as `unknown` by default.

Each tile state is one of:

$$
{\text{unknown},\text{covered},\text{obstacle}}.
$$

### Tile indexing and coordinate mapping

The simulator operates in continuous coordinates for the robot pose $q=[x,y,\theta]^T$, but the BA* planner operates on tiles. We define a deterministic function `pose_to_tile(x, y)` that maps the current robot center to a tile index `(i, j)` and its tile center `(x_i, y_j)` based on the tile resolution (tile side length is the robot diameter $2r$).

(To be completed: exact rounding rule and how tile centers are represented.)

### Covered set and candidate set

For efficient membership tests:

* The covered set $C_{\text{cov}}={s:\hat{M}(s)=\text{covered}}$ is maintained implicitly through $\hat{M}$, and optionally also as a dedicated hash set of covered tile keys.
* The backtracking list $L$ is stored as a set or a de-duplicated list of tile keys, because the same candidate can be detected multiple times during BM.

## Neighbor queries and predicates

### 4-neighborhood and 8-neighborhood

Neighbor sets are computed by fixed integer offsets in tile-index space:

* $N_4(s)$ uses offsets `[(+1,0), (0,+1), (-1,0), (0,-1)]`.
* $N_8(s)$ adds the diagonals.

This neighbor logic is reused in three places:

1. Checking whether BM can proceed.
2. Detecting critical points.
3. Computing the corner score $\mu(s)$ for candidate detection.

### Blocked and uncovered predicates

The algorithm uses the following predicates derived from $\hat{M}$:

* `isBlocked(s)` is true if a tile is already covered or is an obstacle.
* `isUncovered(s)` is true if a tile is still unknown.

In code, these are implemented as state checks on $\hat{M}[s]`with the convention that a missing key implies`unknown`.

## Robot, sensing, and obstacle detection

### Robot state and motion primitives

The simulator maintains the robot pose:

$$
q=[x,y,\theta]^T,
$$

and supports two primitives:

1. Rotate in place by $\alpha$.
2. Translate forward by distance $d$ along heading $\theta$.

Each movement updates the continuous pose and then updates the tile state using `pose_to_tile`.

### Obstacle detection and map updates

Obstacle detection is assumed reliable in the simulator. When the robot observes a tile that is occupied, we set:

$$
\hat{M}(s)=\text{obstacle}.
$$

Collision checks are performed against the obstacle representation before executing a translation segment.

(To be completed: exact sensor footprint and when an obstacle tile becomes observable.)

## Coverage tracking

### Marking covered tiles

A tile is marked `covered` the first time it is swept by BM. This implements the covered-at-most-once invariant:

* BM is the only phase that changes a tile from `unknown` to `covered`.
* Backtracking is allowed to pass through `covered` tiles without changing their state.

### Visited versus covered in code

To avoid mixing concepts:

* `covered` is a persistent tile state in $\hat{M}$.
* `visited` is optional instrumentation (for logging or debugging) that can record how many times a tile is traversed, including during backtracking.

## Walkthrough: Unified scenario in code

This subsection illustrates the control flow using a single consistent scenario. In this unified example, we fix:

* Critical point: $s_{cp}=(50,50)$
* Selected starting point: $s_{sp}=(40,50)$

### Phase 1: Boustrophedon motion (BM)

1. The robot starts BM from an initial pose and repeats:

   * Compute the current tile key $s=\text{pose\_to\_tile}(x,y)$.
   * If $\hat{M}(s)=\text{unknown}$, mark it as `covered`.
   * Evaluate possible next moves using $N_4(s)$ and `isBlocked(·)`.

2. During BM, we also run candidate detection:

   * For tiles in a local window around the robot (or at minimum the current tile), compute the corner score $\mu(s)$ using neighbor states.
   * If $\mu(s)\ge 1$ and $s\in M$, insert $s$ into $L`.

### Phase 2: Critical point detection

At each BM step, we check whether the current tile is a critical point:

* Compute the four neighbors $N_4(s)$.
* If all neighbors are blocked, BM stops and we record the current tile as:

$$
s_{cp}=(50,50).
$$

### Phase 3: Selecting $s_{sp}$ from $L$

Given the candidate set $L$, we select a next starting point.

Implementation options:

1. Euclidean proxy: compute a simple distance from $s_{cp}$ to each candidate.
2. Backtracking cost: for each candidate, run A* (and optionally A*SPT) to estimate the true backtracking distance.

In this unified example, the selected result is:

$$
s_{sp}=(40,50).
$$

### Phase 4: Backtracking path planning (A* then A*SPT)

1. Run A* on the tiling graph whose nodes are covered tiles (plus the current tile), edges connect neighbors, and blocked tiles are excluded.
2. A* returns a discrete tile path:

$$
P=[s_1,s_2,\dots,s_n],
$$

with $s_1=s_{cp}$ and $s_n=s_{sp}$.

3. Apply A*SPT to reduce unnecessary turns and produce a shorter waypoint list:

$$
\hat{P}=[\hat{s}_1,\hat{s}_2,\dots,\hat{s}_k].
$$

(To be completed: line-of-sight collision checking in A*SPT and the exact smoothing rule.)

### Phase 5: Executing the backtracking path

For each consecutive waypoint center $(x_{i+1},y_{i+1})$, we compute the control commands:

$$
\beta=\arctan!\left(\frac{y_{i+1}-y}{x_{i+1}-x}\right),
\qquad
\alpha=\beta-\theta,
\qquad
d=\sqrt{(x_{i+1}-x)^2+(y_{i+1}-y)^2}.
$$

The robot rotates by $\alpha$ and translates by $d$, repeating until it reaches $s_{sp}$.

### Phase 6: Heading adjustment and BM restart

Once the robot reaches $s_{sp}$, it snaps to a discrete heading to restart BM. We implement the priority order:

1. North
2. East
3. South
4. West

Let $\gamma$ be the chosen discrete heading and $\theta$ the current heading. The rotation applied is:

$$
\alpha=\gamma-\theta.
$$

After heading adjustment, the robot re-enters Phase 1 (BM) from $s_{sp}$.

## Deviations and simplifications relative to the BA* paper

(To be completed: list any deviations, simplifications, or implementation-specific assumptions relative to the BA* paper; for example sensing assumptions, movement discretization, or an approximate A*SPT line-of-sight test.)

# Experiments and Evaluation

## Experimental setup

(To be completed with details.) Intended setup:

* Maps represented as binary occupancy grids or binary images (free vs obstacle).
* A circular robot model with radius $r$, operating under the motion primitives defined above.
* Multiple map categories (simple rooms, cluttered environments, narrow passages) to stress both BM and backtracking.

## Metrics

We evaluate BA* using:

1. **Coverage rate:** fraction of reachable free area covered by the robot.
2. **Path length:** total traveled distance (sum of translations).
3. **Number of boustrophedon regions:** number of BM sweep segments separated by backtracking events.

## Results

(To be completed with results.)

* Insert plots of coverage over time.
* Insert path visualizations of trajectories and backtracking paths.
* Insert summary tables across maps: coverage rate, total path length, number of regions.

# Discussion and Limitations

BA* achieves efficient online coverage by combining a sweeping mode (BM) with deliberate backtracking through A* on a discovered tiling graph. The theory in this report emphasizes:

* The distinction between **covered** and **visited** tiles, and the covered-at-most-once invariant.
* The role of a **critical point** as a trigger to switch from sweeping to backtracking.
* The use of A* to return to a promising boundary point, and smoothing (A*SPT) to reduce turn-heavy grid paths.

Limitations highlighted by the theory text include:

* BA* may leave small uncovered portions near obstacle boundaries depending on tiling resolution and boundary handling.
* Online operation means decisions depend on partial knowledge; early choices can influence backtracking frequency and total path length.

## Complexity (report-safe statement)

A report-safe statement is that each A* backtracking computation runs in time polynomial in the number of discovered tiles, and in practice is dominated by graph search on the current tiling graph. The overall runtime depends on the number of backtracking events and the growth of the discovered tiling model during coverage.


# Conclusion

This report describes BA*, an online complete coverage algorithm that performs boustrophedon sweeping most of the time and uses A* on a tiling model for intelligent backtracking when sweeping reaches a critical point. The implementation and evaluation sections are structured to report coverage rate, traveled path length, and the number of boustrophedon regions in simulation.

# References

[1] BA* paper (to be completed).
[2] BCD paper (to be completed).
[3] B-Theta* paper (to be completed).
