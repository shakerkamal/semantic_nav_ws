# UGV Rover, Depth-Only: What Broke and Why

**Scope.** Porting the semantic navigation stack from the TurtleBot3 (Waffle, 360° LiDAR)
to the Waveshare UGV Rover PT running **depth-only** — no LiDAR, `/scan` synthesised from
an RGB-D camera. This is the configuration the real robot will run on the Jetson, so it is
the configuration results must be collected in.

Everything below was hit for real, in simulation, between 2026-06-27 and 2026-07-13. Each
entry gives the **symptom you actually see**, the **real cause**, the **fix**, and the
**evidence** — because in almost every case the obvious explanation turned out to be wrong.

---

## TL;DR — the five that cost the most time

| # | Looked like | Actually was |
|---|---|---|
| 1 | "The map is full of radial spokes" | The laser scan was stamped in an **optical** frame; a LaserScan sweeps its frame's XY plane about **+Z**, and in an optical frame +Z points *forward* — so the scan was a vertical fan. |
| 2 | "Spinning corrupts the map" | **One** false loop closure out of 36, and robust graph optimisation was switched off, so that single bad constraint dragged the entire map. |
| 3 | "The robot drifts away from the door" | It wasn't drifting, it was **rotating** away: Nav2's `Spin` overshoots, and the re-observe sweep had two spins one way and one the other, so the overshoot never cancelled. |
| 4 | "It needs to spin to see the door open" | It didn't. RTAB-Map's map was **frozen while parked** (`map_always_update=false`). The spin was never a perception strategy — it was a trick to force a new graph node. |
| 5 | "Navigation fails on sharp turns" | The rotation controller turns **in place** (0 m of translation), and the progress checker only counted translation, so it declared the robot stuck and aborted a path it was following perfectly. |

**The recurring lesson:** on this platform, the *sensor* and the *robot* are honest — it is
almost always a **frame, a parameter type, or a watchdog** that is lying.

---

## Why depth-only at all

The real rover carries an **OAK-D Lite** and **no LiDAR**. Waveshare's simulation model
ships a 360° `hls_lfcd_lds` (3.5 m) that the hardware does not have. Results collected with
that LiDAR would not transfer to the Jetson, so the sim rover was rebuilt to sense exactly
what the robot will:

- our own copy of the rover model with the LiDAR sensor **removed**
- `/scan` synthesised from the depth image via `depthimage_to_laserscan`

**The cost is real and unavoidable: horizontal field of view collapses from 360° to 59°.**
The rover is genuinely blind to its sides and rear. Several problems below are only
problems *because* of that, and they will all exist on the hardware too.

---

## 1. Bring-up

### 1.1 The rover spawns invisible
**See:** the house loads, the robot does not appear (but its topics exist).
**Cause:** `model.sdf` refers to its meshes as `model://ugv_description/meshes/...`.
`GAZEBO_MODEL_PATH` needs **both** `ugv_gazebo/models` (for the model) **and the parent of**
`ugv_description/share` (for the meshes). Miss the second and the model spawns with no geometry.
**Fix:** build `GAZEBO_MODEL_PATH` explicitly in `aws_small_house_ugv.launch.py`.

### 1.2 Teleop publishes but the robot does not move
**Cause:** not a bug. `teleop_twist_keyboard` uses **`i` `j` `k` `l` `,`**, not WASD or arrows.
Cost an hour and a wrong hypothesis about strafing before anyone actually read the key map.

### 1.3 A stale teleop node silently drives the robot
**See:** the rover creeps away from where you left it; a pose reset does not hold.
**Cause:** `gazebo_ros_diff_drive` has **no command timeout**. A `teleop_twist_keyboard` left
running from an earlier session keeps applying its last twist *forever*.
**Fix:** kill stray teleop nodes. Worth knowing because it looks exactly like a spawn or
odometry bug — we watched the rover walk from (2.97, −0.81) to (2.91, −1.09) with nothing
apparently commanding it.

### 1.4 Custom BT nodes are "not recognized"
**Cause:** `bt_navigator.plugin_lib_names` **replaces** Nav2's default list, it does not
append to it. Supplying a short list silently drops every standard BT node.
**Fix:** full 47-entry default list **plus** `semantic_nav_nav2_plugins`.

### 1.5 RTAB-Map starves on approx-sync and Nav2 never activates
**See:** `Did not receive data since 5 seconds!` even though every input topic is publishing;
no `/map`; `bt_navigator` stuck `inactive`; goals rejected.
**Cause:** `/odom` publishes at **~100 Hz** while the camera runs at ~17 Hz, and rtabmap's
`topic_queue_size` is **10** — barely 0.1 s of odom history. A camera frame can arrive after
its matching odom has already been evicted, and the sync never recovers.
**Status:** understood, not yet fixed. If a bringup hangs with no `/map`, this is it.

---

## 2. Mapping (RTAB-Map)

### 2.1 The map is a fan of radial spokes  *(the big one)*
**See:** walls render as long lines radiating from the robot instead of walls.

**Cause:** `depthimage_to_laserscan` was told to publish in `3d_camera_link`. That link is an
**optical frame** — `RPY(−90°, 0, −90°)`, so **+Z points forward**, +X right, +Y down. But a
`LaserScan` is *defined* to sweep its frame's **XY plane, rotating about +Z**. Stamp a scan
into an optical frame and you have declared a **vertical fan rotating about the forward
axis**. Flatten that into a 2D grid and you get spokes. The scan itself was perfect
(58.9° span, 640 rays, sane ranges) — only the frame was wrong.

`depthimage_to_laserscan` expects a **non-optical** output frame; its own default is
`camera_depth_frame`, deliberately not `..._optical_frame`.

**Fix:** publish `camera_scan_link` at the camera's position with the optical rotation undone
(conjugate quaternion), and emit the scan there. Verified numerically:
`base_footprint → camera_scan_link` is exactly the **identity** rotation.

### 2.2 The occupancy grid can mark but never clear
**Cause:** Waveshare ships `Grid/RayTracing=false`. A grid that can mark cells occupied but
never carve free space can only ever get **denser**. Also `Grid/3D=true` (a 3D grid projected
down, not a 2D nav grid) and `Grid/RangeMax=0` (unlimited).
**Fix:** `Grid/RayTracing=true`, `Grid/3D=false`, `Grid/Sensor=0`, bounded `Grid/RangeMax`.

> **Naming trap:** `Grid/FromDepth` is the **old** parameter name and is **silently ignored**
> by this rtabmap build. It uses `Grid/Sensor`. Setting the old name looks like it worked and
> does nothing.

### 2.3 rtabmap dies instantly with no output (SIGABRT, exit −6)
**See:** the node starts and dies in 0.5 s having printed **nothing**. No `/map`, Nav2 stuck.

**Cause:** rtabmap declares **every `Grid/*` parameter as a string**. Passing one from a
`LaunchConfiguration` lets launch_ros helpfully auto-convert `"3.5"` into a **double**, and
rtabmap throws `InvalidParameterTypeException` and `terminate`s before logging anything.

**Fix:** `ParameterValue(value, value_type=str)`. The same trap bit `Optimizer/Robust` later.

### 2.4 The map overlaps itself — rotated, doubled copies
**See:** after a few circuits the house appears twice, rotated against itself.

**Cause:** **exactly one false loop closure.** Reading the pose graph out of rtabmap's SQLite
DB and checking every loop closure against ground truth:

```
36 global loop closures
35 excellent          -> median error 1.0 cm
 1 catastrophic       -> node 266 -> 187 : 299.7 cm, 10.4 deg off truth
```

`Optimizer/Robust` was **false**, so that single bad constraint dragged the whole graph:
`map→odom` was measured at **(0.33, −2.36 m, 14.7°)** when a correct solution must hold it at
~identity (odom is ground truth in sim).

**Why depth-only makes this worse:** a 59° FOV sees less of each place, so bag-of-words
mismatches between near-identical white-walled rooms are far likelier.

**Fix:** `Optimizer/Robust=true`. GTSAM was already linked and `Optimizer/Strategy=2` — robust
optimisation was available the whole time, just switched off. It down-weights outlier
constraints instead of believing them.

### 2.5 The sensing horizon was a leftover from a sensor we deleted
**Cause:** `3.5 m` appeared in three places (`range_max`, `Grid/RangeMax`, costmap ranges).
It was Waveshare's **LiDAR** ceiling — `<max>3.5</max>` — and in depth-only **there is no
LiDAR**. In a ~15 × 12 m house a 3.5 m horizon cannot see across one room.

**This is what made obstacles uncleanable.** Clearing requires a ray to pass *through* a cell
and **terminate on something**. With nothing within 3.5 m, the rays came back **NaN**
(measured: **217 of 217** straight ahead), and a NaN ray traces nothing.

**Fix:** raise the depth scan and `Grid/RangeMax` together; let costmap **clearing reach
further than marking** (`raytrace_max_range` > `obstacle_max_range`).

> **This constraint does not go away.** A blocker in front of open space beyond sensor range
> can *never* be cleared, no matter how long you dwell or how often you spin.

---

## 3. Motion and control

### 3.1 Inflation seals the doorway
`inflation_radius: 0.55` leaves **zero** free cells in the ~0.9 m doorways. The planner can
never route through, even with the door open, which makes every blockage recovery
**unfalsifiable**. Set to `0.35`. The tell is `barrier=cleared plan_ok=False`.

### 3.2 The velocity smoother throttled everything
**Cause:** the smoother's angular limits were copied from Waveshare's **DWB** block
(`acc_lim_theta: 0.5`, `decel_lim_theta: −0.5`). Those are DWB **trajectory-sampling** limits
for path following, **not actuator limits** — and the smoother sits directly in the `cmd_vel`
path. It capped angular deceleration at 0.5 rad/s² while `behavior_server`'s `Spin` plans with
`rotational_acc_lim: 3.2`. So when Spin said *stop*, the smoother took **2 seconds** to wind
down, sweeping up to **57°** past target. A 40° spin turned 68°.

**Rule:** *a velocity smoother must never be more restrictive than the behaviours commanding
through it, or it prevents them from stopping.*

### 3.3 Navigation aborts on sharp turns  *(the one that fails a demo)*
**See:** the planner finds a path, the robot barely moves, Nav2 aborts. Teleop it straight,
re-issue the *same* goal, and it works.

**Cause:** the rover drives behind a `RotationShimController`. Whenever the path needs more
than 45° of heading change, the shim rotates **in place** — moving **0 m**.
`SimpleProgressChecker` only counts **translation**, so it read that as *stuck*. The rover is
skid-steer and delivers only ~0.47× commanded rotation, so a 180° turn burns ~6.7 s at zero
translation and then still needs ~1.9 s to cover the required 0.5 m — **8.6 s of a 10 s
budget**. Any replan mid-turn tips it over.

**Evidence** (`open_set_ugv_system_20260713_205333`) — the same goal, three attempts:

| attempt | pose | `Failed to make progress` | path found | result |
|---|---|---|---|---|
| from the bed | **angled** | **3×** | `path_length=10.862` | ABORTED |
| retry | **angled** | **6×** | yes | ABORTED |
| after teleop | **straight** | **0×** | `path_length=10.866` | **SUCCEEDED** |

The planner returned **the same path** every time. DWB never failed to find a trajectory.
The **only** variable was the robot's heading.

**Fix:** `PoseProgressChecker`, which accepts **rotation *or* translation** as progress
(`required_movement_angle`). An in-place turn is no longer mistaken for being stuck.

> This one was self-inflicted: the progress checker was added while aligning the rover's Nav2
> params with the TB3's, without noticing that the rover drives behind a rotation shim and the
> TB3 does not.

### 3.4 The rover's real rotation is nothing like the commanded one
Measured, closed-loop: **commanded 344°, delivered 163°** — under half. Four-wheel skid-steer
must *scrub* its wheels sideways to rotate. It also **coasts 7–17° after the command stops**,
roughly independent of how far it was asked to turn. Two hypotheses for that coast were tested
and **both were wrong** (`max_wheel_acceleration` — raised 20×, no change; smoother
deceleration — raising it made overshoot *worse*). The chassis simply skids to a halt. Treat
the coast as a fact of the platform, not something to tune away.

---

## 4. Recovery

### 4.1 The re-observe spin rotated the robot away from the door
**See:** after a few `approach_and_recheck` rounds the rover is facing the wrong way entirely.

**Cause — structural, not tuning.** Nav2's `Spin` overshoots in **whichever direction it is
turning**. The sweep was `(+y, −2y, +y)` — **two positive spins and one negative** — so with a
per-spin overshoot δ the net is `(y+δ) + (−2y−δ) + (y+δ) = +δ`. **The overshoot survives once,
every single sweep**, and compounds.

Measured: **+13° per re-observe on average (worst +27°) → ~120–140° after five rounds.**
Translation through a whole sweep was **0.7 cm** — the rover was never drifting, only rotating.

### 4.2 …and the spin was never needed in the first place  *(the good one)*
**Cause:** rtabmap's `map_always_update` defaults to **false**, which refreshes the occupancy
grid **only when a new graph node is added** — and a node requires ≥ 0.1 m or ≥ 0.1 rad of
**motion**. **A stationary robot's map is frozen**, however long it stares at a doorway that
has just opened. The spin was never a perception strategy; it was a **trick to force node
creation**.

**Verified** by spawning a box in front of a **parked** rover and deleting it:

| | `/map` | global costmap |
|---|---|---|
| `map_always_update=false` (stock) | box **never even appears** | 47 cells, never clears |
| `=true`, open space | appears ✓ | never clears *(NaN rays — see 2.5)* |
| `=true`, wall behind the box | appears ✓ → **clears in ~10 s** | **clears in ~20 s** |

**The robot never moved.** With `map_always_update=true` + `Grid/RayTracing`, a removed
blocker clears from both layers **with no spin at all**.

**Fix:** re-observe by **dwelling** — the standoff already faces the barrier
(`compute_standoff` sets `yaw = atan2` toward it), so rotating buys nothing. Spin only *once*,
as a full 2π turn, if the dwell fails. Modes: `dwell_then_spin` (rover), `spin` (TB3).

### 4.3 The shared orchestrator nearly broke the TurtleBot3
The orchestrator is **shared** with the TB3 stack, whose launch sets no re-observe params and
whose rtabmap (from `rtabmap_demos`) has **no `map_always_update`**. Defaulting to
`dwell_then_spin` would have had the TB3 dwell 12 s against a **frozen map** and then do one
2π turn instead of its evaluated sweep — silently invalidating locked Ch7 results.

**Rule:** *defaults belong to the incumbent.* The orchestrator's defaults reproduce TB3's
original behaviour exactly; the rover **opts in** from its own launch.

---

### 4.4 The blockage centroid landing in the wrong place  *(did not reproduce; likely fixed)*
`open_set_ugv_system_20260713_230141` — the centroid came out at **(−2.455, +0.755)** where the
partition truly sits near **(−2.507, −1.350)**: same x, **2.07 m out in y**. The rover then
verified empty floor and honestly reported the barrier clear.

The likely cause was found on 2026-07-14, and it was not SLAM drift. `EscalateToLLMRecovery` and
`QuerySemanticContext` read the robot pose from the blackboard key `robot_pose` and, when it was
absent, **fell back to the `goal` key**. Nav2's `NavigateToPose` tree never sets `robot_pose`, so
that fallback fired *every single time* — the semantic context was queried around the
**destination**, not the robot. That displaces the query centre by the robot-to-goal distance,
which looks exactly like a wandering centroid. Both nodes now read the live TF pose and fail
loudly if TF cannot supply it. On 2026-07-14 the centroid came out at (−2.479, −1.378) against a
door at (−2.51, −1.375). Watch it, but stop blaming SLAM.

---

## 5. Motion and planning: two bugs that both look like "it can't fit through the gap"

**The gap was innocent both times.** Measured: **0.95–1.00 m** in `/map`, **0.75–0.80 m** of
costmap corridor, and the rover is **0.2 m wide**. The TurtleBot3 is **2.2× fatter**
(`robot_radius` 0.22 vs 0.1) with identical inflation, and threads the same doorway happily.

### 5.1 DWB could not turn  *(FIXED)*
Parked beside the doorway, the controller was emitting:

```
cmd_vel_nav:  linear.x = 0.0        ← every forward trajectory rejected
              angular.z = -0.0192   ← ~80 seconds to turn one radian
```

DWB only samples velocities reachable **from the currently measured one within a control
cycle**. With `acc_lim_theta: 0.5` at 10 Hz that window is **±0.05 rad/s** — so 0.019 rad/s was
not a choice, it was the *only* thing on the menu. Then the skid-steer closes a trap: DWB
commands 0.019, the chassis under-delivers (**0.006 rad/s** measured), the next cycle samples
±0.05 around *that*, and the rotation can **never ramp up**. It creeps until the progress
checker aborts.

`acc_lim_theta` 0.5 → **3.2**, `decel_lim_theta` −0.5 → **−3.2**, `decel_lim_x` −0.5 → **−2.5**.

The irony: we had *already* diagnosed these Waveshare numbers as DWB **sampling** limits and
removed them from the velocity smoother — and left them in DWB itself, where they are a hard cap
on what it can command. `behavior_server`'s `Spin` drives the same chassis fine at
`rotational_acc_lim: 3.2`; DWB deserved the same authority.

### 5.2 NavFn could not extract a path  *(FIXED — planner swapped)*
```
[ERROR] Failed to create a plan from potential when a legal potential was found.
        This shouldn't happen.
```
Read the message literally: **a legal potential was found**, i.e. Dijkstra reached the goal and
**a path provably exists**. (An independent flood-fill agreed: goal reachable, 56 854 cells.)
What failed is NavFn's *second* stage — it extracts the path by walking the **gradient** of that
potential field, and the gradient descent stalled.

Why here and not on the TurtleBot3? **Sensing density, not geometry.** The 59° depth FOV leaves
**unknown cells scattered through open floor** the cone never swept — visible in a costmap probe
as `?` in the middle of a room. NavFn does not treat unknown as free:

```cpp
} else if (v == COST_UNKNOWN_ROS && allow_unknown) {
  v = COST_OBS - 1;      // = 253, one below LETHAL
}
```
Free space is `COST_NEUTRAL` = **50**. Every unknown cell becomes a **253-cost spike** — a
near-lethal pillar standing in open floor. The potential field fills with local minima, and the
gradient walks into one. A 360° LiDAR leaves no such holes, which is precisely why TB3 never hit
this with the same planner.

Fix: **`nav2_smac_planner/SmacPlanner2D`** — real A* on the cost grid, path emitted directly, no
gradient-descent stage to fail. Upstream considers NavFn legacy for exactly this reason (the
`nav2_smac_planner` README cites "the weird artifacts introduced by the gradient wavefront-based
2D A\* implementation in the NavFn Planner"). **TB3 keeps NavFn.**

*Worth running:* relaunch the rover with `depth_only:=false` (sim LiDAR) but NavFn restored. If
NavFn then plans fine, the sensing-density explanation is confirmed outright — same robot, same
world, same planner, only the FOV changed.

### 5.3 A stale teleop that fakes a controller bug  *(process trap — check this FIRST)*
`teleop_twist_keyboard` publishes **zero twists at ~10 Hz** even when no key is pressed. It
races `velocity_smoother` on `/cmd_vel`, and `gazebo_ros_diff_drive` has **no cmd_vel timeout**,
so a teleop you opened half an hour ago keeps voting forever. Nav2's commands get interleaved
with zeros, the rover twitches, and it stalls.

It presents as **"the robot can't do sharp turns"** — because a rotate-in-place is the manoeuvre
most easily cancelled by interleaved zeros, while straight-line driving coasts through a few
dropped commands. This has cost a full debugging detour **three times**.

**Pre-flight, before any trial:**
```bash
ros2 topic info /cmd_vel -v | grep "Publisher count"   # velocity_smoother + behavior_server ONLY
```

---

## Still open

### Removed obstacles reappear in `/map`
The rover opens a door, the dwell clears it, navigation succeeds through the gap — and then the
barrier is **back**, blocking a doorway the robot physically drove through.

The barrier lives in **`/map` (RTAB-Map's static grid)**, not in the costmap obstacle layer —
verified by probing `/map`, `/global_costmap` and `/local_costmap` at the same world point. Nav2
is innocent; it faithfully inflates a wall RTAB-Map told it about. **Do not go hunting
costmap-clearing bugs.**

RTAB-Map assembles the global grid from **per-node local grids**, which are immutable. Nodes
recorded while the door was shut still carry those cells as occupied. `map_always_update=true`
clears them in the **live assembly** (which is why the dwell genuinely works) — but the next
**graph optimization regenerates the grid from the cached local grids** and the old cells come
straight back. The clearing was never durable.

RTAB-Map's own tool says so: *"Clear empty space from local occupancy grids … If the map needs to
be regenerated in the future, **removed obstacles won't reappear**."*

The fix, **verified live but deliberately not wired**:
```bash
ros2 service call /rtabmap/cleanup_local_grids \
  rtabmap_msgs/srv/CleanupLocalGrids "{radius: 1, filter_scans: false}"
```
**Timing is the whole trick:** it only clears cells the *current* assembled map shows as EMPTY.
Called at the right moment the doorway reopened (1.00 m in `/map`, 0.80 m in the costmap); called
after the map re-sealed it is a **no-op returning `modified: 0`**. It belongs immediately after
the barrier-clear confirmation. On the Jetson it needs care — the service warns that with enough
drift it can erase a **real** wall.

### The barrier-clear gate is a no-op  *(known; left as-is)*
It always returns `cleared`. This is arithmetic, not a race: a door is one cell thick, so inside
the 11×11 sampling window it can contribute at most one column — **11 of 121 cells = 0.091** —
against a **0.15** threshold. 0.091 is the mathematical *ceiling* for any 1-cell barrier, so
`still_blocked` is unreachable. Reproduced on the live costmap with the door physically shut and
correctly mapped in all three grids.

Consequence: **`plan_ok` does 100% of the real gating and `barrier_ok` is always `True`.** Runs
still succeed — but **never cite "the robot confirmed the barrier was clear" as evidence in a
trial.** A fix must be **scale-invariant**: `clear_radius` is `max(barrier_clear_radius_m,
barrier_extent_m/2)` and therefore *grows with the barrier*, so a naive absolute lethal-cell
count breaks too (tried, reverted — it read `still_blocked` forever on wide barriers).

### Others
- **RTAB-Map approx-sync starvation** (§1.5) — understood, not fixed.
- **The velocity smoother still caps angular at 1.0 rad/s** while the rotation shim asks for
  1.8. Left alone deliberately so the `PoseProgressChecker` fix stays attributable.
- The rover can end up **facing away from the blocker** after the dwell (59° FOV).

---

## 6. The remote LLM link (zenoh)

### 6.1 The llama action server is visible but never answers a goal  *(FIXED)*
**See:** `ros2 action info /llama/generate_response` shows the server, but `send_goal` gets no
reply. Every BT recovery proposal returns `LLM recovery call failed or timed out`. It "breaks
whenever a new node appears".

**Cause:** the bridge was started **without `-c zenoh-local.json5`**, so its allow-list —
`action_servers/action_clients: ["/llama/.*"]`, i.e. *"action = the only thing crossing the
bridge"* — was never applied. Without it `zenoh-bridge-ros2dds` routes **every entity of the
~25-node stack**, and `tcp/127.0.0.1:7447` is an **SSH tunnel** to the remote llama server. Every
new node rebuilt its routing table and tore down the live query route. `ros2 action info` still
worked because it reads *cached graph discovery*; sending a goal needs the *live route*.

**Fix (confirmed working):**
```bash
cd ~/Thesis/Implementation/demo_bringup/zenoh
./zenoh-bridge-ros2dds -c zenoh-local.json5      # -c is mandatory, and this is the WHOLE command
```
Do **not** also pass `-d 42 -e tcp/... --no-multicast-scouting client`: the config already sets
domain, endpoint and scouting, and the positional `client` contradicts its `mode: "peer"`.

**Run exactly ONE bridge.** Two on the same domain both route the action, so goals get raced or
duplicated — which looks exactly like flaky intermittent failure. `pgrep -f zenoh-bridge-ros2dds`
must return one.

> Worth internalising: the LLM timeouts in the 2026-07-14 log were **not** an LLM problem and
> **not** a rover problem. They were transport. A component that reports "timed out" is telling
> you where it gave up, not where the fault is.

---

## Quick reference

| Symptom | Look here first |
|---|---|
| Map is radial spokes | Scan `output_frame` is an **optical** frame (§2.1) |
| Map overlaps / doubles | `Optimizer/Robust=false` + one false loop closure (§2.4) |
| rtabmap dies instantly, no log | `Grid/*` param passed as non-string (§2.3) |
| Obstacles never clear | `Grid/RayTracing=false`, or NaN rays past `Grid/RangeMax` (§2.2, §2.5) |
| Map never updates while parked | `map_always_update=false` (§4.2) |
| `barrier=cleared plan_ok=False` | Verifying the wrong place, or inflation sealing the gap (§3.1, Still open) |
| Aborts on sharp turns | `SimpleProgressChecker` vs `RotationShimController` (§3.3) |
| Robot ends up facing backwards | Unbalanced spin sweep (§4.1) |
| Robot creeps on its own | Stale `teleop_twist_keyboard` (§1.3) |
| Nav2 stuck `inactive`, no `/map` | rtabmap approx-sync starvation (§1.5) |
| llama action visible but goals unanswered | zenoh bridge started without `-c` (§5.1) |

---

## What this cost us, and what to carry to the Jetson

**Simulation flatters the SLAM.** `gazebo_ros_diff_drive` defaults `odometry_source` to
**WORLD**, so `/odom` in sim is essentially **ground truth** — verified to 0.0° and 0.000 m
through a full spin. RTAB-Map is being handed a perfect motion prior. That is why `map→odom`
sits frozen at rest and why the map looks as good as it does with a 59° FOV.

**On the real rover none of that holds.** Wheel odometry will drift, the IMU is genuinely
noisy and *will* matter (it feeds `robot_localization` in `bringup_imu_ekf`, whereas in sim it
is wired to nothing and rtabmap ignores it via `Optimizer/GravitySigma=0`), and registration
will have to do real work. **The sim map is an optimistic upper bound, not a preview.**

The habits that actually paid:

1. **Measure the thing, do not reason about it.** Nearly every confident first diagnosis here
   was wrong — the odometry drift theory, the wheel-acceleration theory, the depth-cloud-grid
   theory. Each died on contact with a number.
2. **Check what the node *applied*, not what you *asked for*.** `Grid/FromDepth` was ignored;
   `Grid/Sensor` was in force. `ros2 param get` on the live node settles it.
3. **Change one thing.** Two changes at once make a fix unattributable — and twice here the
   "obvious" second change turned out to make things *worse*.
4. **Defaults belong to the incumbent.** A shared component's defaults must keep the existing
   platform byte-identical; the newcomer opts in.
