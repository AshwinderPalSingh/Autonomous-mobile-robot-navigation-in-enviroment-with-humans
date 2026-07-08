# Design Notes — Social-Aware Navigation

This document explains *why* the system is built the way it is, and derives the
math behind the custom costmap layer. It is written to double as interview prep:
each section ends with the point worth making out loud.

---

## 1. System philosophy: classical baseline first

The project deliberately builds a **complete, measured classical stack** (Nav2
with NavFn global planning + DWB local control) and extends it with **one**
custom component: a social costmap layer. Nothing about the navigation is
learned. That is a choice, not a limitation:

- A classical baseline is reproducible and debuggable. When the robot does
  something, you can point at the costmap cell and the critic score that caused
  it.
- It gives a **measurable reference** to compare against. The A/B protocol
  (§6) quantifies exactly what the social layer changes, in metres and seconds.
- Learned social navigation (trajectory prediction, RL policies) is the honest
  *next* step, layered on a validated baseline rather than replacing it.

> **Talking point:** "I built the systems-engineering foundation and validated
> it with metrics, then extended it with one social-navigation component. The
> learned version is future work, benchmarked against this baseline."

---

## 2. Why pedestrians are a ROS node, not Gazebo actors

Detections are published by `pedestrian_sim` as ground truth
(`social_nav_msgs/Pedestrians`: id, position, velocity). This node plays the
role of a **perfect perception stack**.

Reasons, in order of importance:

1. **Clean separation of concerns.** The project's contribution is the
   *navigation-side* integration. Perception (leg detection, vision tracking)
   is a different, large problem; simulating it as ground truth keeps the
   contribution crisp and swappable — a real detector that emits the same
   message drops in without touching the plugin.
2. **The A/B demo is unambiguous.** With the layer disabled, the baseline stack
   *cannot see the pedestrians at all* (they aren't lidar obstacles), so it
   drives straight through them. That is the starkest possible evidence that
   the social layer — and nothing else — produces the avoidance.
3. **Platform reality.** Ignition Fortress actors have no collision physics and
   Humble's `ros_gz` has no pose-teleport service bridge, so scripted physical
   actors would be awkward anyway.

> **Talking point:** "Perception is simulated as ground truth on purpose — it
> isolates my contribution and makes the A/B comparison clean. The interface is
> a message, so a real detector swaps in later."

---

## 3. The cost model (the core deliverable)

Each pedestrian is painted into the costmap as a **velocity-scaled, asymmetric
2-D Gaussian** plus a **lethal core**. This is the Kirby proxemics model — the
accepted classical baseline for social navigation.

### 3.1 Asymmetry from proxemics

People need more clearance *in front of* a moving person than beside or behind
them. So the Gaussian's standard deviation depends on direction relative to the
pedestrian's velocity **v**:

```
sigma_front = sigma_base * (1 + speed_factor * |v|)     # ahead of travel
sigma_back  = sigma_base                                # behind
sigma_side  = sigma_base * sigma_side_ratio             # lateral (< base)
```

`sigma_front` grows with speed: a fast walker's "keep clear" zone stretches
further ahead, so the planner routes around **where the person is going**, not
just where they are. That is anticipatory avoidance with **no prediction
machinery** — the velocity term encodes it directly.

### 3.2 Evaluating a cell

For a costmap cell at world offset `(dx, dy)` from the pedestrian, rotate into
the pedestrian's velocity-aligned frame (heading `theta = atan2(vy, vx)`):

```
lx =  cos(theta)*dx + sin(theta)*dy      # along direction of travel
ly = -sin(theta)*dx + cos(theta)*dy      # lateral
```

Pick the longitudinal sigma by sign of `lx`, then evaluate the Gaussian:

```
sx   = (lx >= 0) ? sigma_front : sigma_back
cost = amplitude * exp( -0.5 * ( lx^2 / sx^2 + ly^2 / sigma_side^2 ) )
```

A **standing** pedestrian (`|v| < min_moving_speed`) has no defined heading, so
the layer falls back to an isotropic Gaussian with `sigma_base` — no rotation,
no asymmetry.

### 3.3 Lethal core vs. passable field

- Cells within `lethal_radius` of the body → `LETHAL_OBSTACLE` (253). No plan
  ever routes through a person.
- The Gaussian amplitude is **clamped below** `INSCRIBED_INFLATED_OBSTACLE`
  (253) — deliberately *costly but passable*. If a pedestrian blocks the only
  corridor, the robot yields and waits rather than declaring the goal
  unreachable and dead-locking. That is the difference between "polite" and
  "brittle".

### 3.4 Merge policy

Costs merge into the master grid with `max()` (never lowering existing cost),
so the social layer only ever *adds* caution and can't erase a real lidar
obstacle. Contributions below `cutoff` are skipped entirely, which also bounds
the per-pedestrian update footprint (see `affectedRadius`).

> **Talking point:** "The velocity term is the whole trick — it stretches cost
> ahead of a moving person, so the global plan bends around their future
> position without any explicit trajectory prediction."

---

## 4. Integration details that make it production-flavoured

These are the details that separate a real Nav2 layer from a tutorial:

- **TF at update time.** Detections arrive in some frame (here `odom`) and are
  transformed into the costmap's global frame every `updateBounds`. Position is
  transformed as a point; **velocity as a free vector** (rotation only).
- **Staleness timeout.** Detections older than `keep_time` are dropped, not
  frozen in place — a stale reading must not drive a physical action. (This is
  the same "no unverified/expired input drives a consequential action" principle
  that governs the rest of the stack.)
- **`current_` always true.** The layer reports current even when empty or
  disabled; forgetting this makes Nav2 treat the *entire* costmap as stale.
  A classic, non-obvious Nav2 gotcha.
- **Thread safety.** The subscriber callback and the costmap update run on
  different threads; shared state is mutex-guarded.
- **Live tunability.** Every parameter is dynamic (`ros2 param set`), so the
  layer can be toggled and tuned mid-demo without a restart.

> **Talking point:** "The three things people get wrong in a custom Nav2 layer
> are transforming the detections, reporting `current_`, and thread safety — I
> handle all three."

---

## 5. Frame and TF authority

One transform, one owner — no arbitration ambiguity:

```
map  -> odom             slam_toolbox (mapping) OR AMCL (missions)
odom -> base_footprint   Ignition DiffDrive plugin (bridged onto /tf)
base_footprint -> ...    robot_state_publisher (URDF)
```

`base_footprint` is the single robot frame every component agrees on
(`robot_base_frame` in Nav2, `base_frame` in slam_toolbox, `child_frame_id` in
DiffDrive). The robot spawns at the world origin, so `odom` coincides with world
coordinates and pedestrian waypoints stay human-readable.

> **Talking point:** "Exactly one node owns each transform. `map->odom` comes
> from SLAM or AMCL but never both at once."

---

## 6. Evaluation: the A/B protocol

The same delivery mission is run twice, identical except for one toggle:

```
# Baseline: social layer off in both costmaps
ros2 param set /global_costmap/global_costmap social_layer.enabled false
ros2 param set /local_costmap/local_costmap  social_layer.enabled false

# Social: on (default)
```

`metrics_logger` records, per run: minimum robot-pedestrian distance,
personal-space intrusions (< 0.8 m), path length, and duration. The expected
result — quantified, not just filmed:

| Metric                     | Baseline | Social |
|----------------------------|----------|--------|
| Min pedestrian distance    | low (near/through) | higher (yields) |
| Personal-space intrusions  | high     | low    |
| Path length                | shorter  | slightly longer |
| Duration                   | shorter  | slightly longer |

The trade is intentional and is the point: the social config accepts a small
path/time cost to buy a large reduction in personal-space intrusion.

> **Talking point:** "I didn't just film a reroute — I measured the trade-off.
> The social layer cuts personal-space intrusions substantially for a few
> percent more path length."

---

## 7. Honest limitations & future work

- **Perception is ground truth.** No real detection/tracking. Next: leg
  detector or vision-based tracker publishing the same message.
- **Constant-velocity assumption.** Cost is shaped by *current* velocity; there
  is no multi-step trajectory prediction. Next: a learned predictor feeding
  per-pedestrian cost, which is the ORACLE-Nav / SoNav research direction this
  project is a scoped-down, shippable version of.
- **Single robot.** The title concept is multi-robot; this is one robot plus a
  delivery mission manager. Multi-robot needs namespaced Nav2 stacks and an
  inter-robot cost channel — documented as future work.
- **"State of practice," not "state of the art."** The Kirby model is the
  accepted classical baseline; current SOTA is learned prediction. Framing it
  honestly is stronger than overclaiming.

> **Talking point:** "This is the validated classical baseline. The SOTA
> extension — learned trajectory prediction — is exactly my research direction,
> and this gives me the measured reference to prove it beats the baseline."
