# Porter — Project Plan

Working name: **Porter** (Traxxas Slash + grocery wagon → garage → apartment).

Mission: a small ackermann robot that can tow (or later push) a loaded grocery wagon from the car in the garage up to the apartment door, fully autonomously, with room to grow into estimation, SLAM, planning, and ML.

---

## 1. Naming

### Recommendation: **Porter**

Short, readable, and maps cleanly to ROS 2 packages:

| Today | After rename |
| --- | --- |
| repo `f1tenth_workspace` | `porter` or `porter_ws` |
| workspace `~/f1tenth_ws` | `~/porter_ws` |
| `f1tenth_control` | `porter_control` |
| `f1tenth_drive` | `porter_drive` |
| future packages | `porter_localize`, `porter_nav`, `porter_perception`, `porter_bringup` |

“Porter” means someone who carries luggage — fits groceries-to-door without sounding like a racing stack. It stays meaningful even if the hitch design changes (tow vs push vs drive-under).

### Strong alternatives

| Name | Vibe | Package prefix | Tradeoff |
| --- | --- | --- | --- |
| **Cartos** | playful, ROS-y | `cartos_*` | Fun; slightly abstract |
| **Bellhop** | carries bags to your door | `bellhop_*` | Evocative; longer to type |
| **Towpath** | hitch + path following | `towpath_*` | Great if towing sticks; awkward if you abandon the hitch |
| **SlashCart** | keeps Slash hardware identity | `slashcart_*` | Ties the brand to one chassis |
| **Homehaul** | destination-focused | `homehaul_*` | Clear mission; a bit corporate |

**Suggestion:** commit to **Porter** unless you strongly prefer another. Do the mechanical rename once, after the first hitch pull-test confirms the vehicle concept is worth keeping.

Node names like `slash_mpc` can stay temporarily (hardware-specific) or become `porter_path_mpc` in the same rename PR.

---

## 2. How to set up the rename

Do this as **one dedicated PR**, separate from behavior changes, so git history stays easy to review.

### 2.1 Decide the brand string

Pick one lowercase identifier used everywhere as the package prefix, e.g. `porter`.

### 2.2 Rename ROS packages (in-repo)

For each package under `src/`:

1. Rename the directory: `f1tenth_control` → `porter_control`, `f1tenth_drive` → `porter_drive`.
2. Update `package.xml` `<name>`.
3. Update `CMakeLists.txt` `project(...)`.
4. Move/rename include tree: `include/f1tenth_control/` → `include/porter_control/`.
5. Fix `#include "f1tenth_control/..."` and any `FindPackageShare("f1tenth_...")` / launch `package=` strings.
6. Optionally rename executables/nodes:
   - `slash_mpc` → `porter_path_mpc` (or keep `slash_mpc` until control changes)
   - `slash_twist_to_ackermann` → `porter_twist_to_ackermann`
7. Update YAML root keys to match the node name (`porter_path_mpc:` etc.).
8. Search the repo for leftovers: `rg -n 'f1tenth|slash_mpc|slash_twist|f1tenth_ws'`.

Rebuild cleanly:

```bash
cd ~/porter_ws   # after folder rename, or stay on old path until then
rm -rf build install log
colcon build --event-handlers console_direct+
source install/setup.bash
ros2 pkg list | grep porter
ros2 launch porter_control slash_mpc.launch.py   # or new launch name
```

### 2.3 Workspace folder vs GitHub repo

These are independent:

| Layer | Action |
| --- | --- |
| Local folder | `mv ~/f1tenth_ws ~/porter_ws` (update shell aliases / README paths) |
| Git remote | GitHub → Settings → Rename repository (e.g. to `porter`). GitHub redirects the old URL. |
| Clone remotes | `git remote set-url origin git@github.com:<user>/porter.git` |

No need to rewrite git history.

### 2.4 What *not* to rename yet

- Topic contracts (`/odom`, `/ackermann_cmd`, `/cmd_vel`) — keep stable so bags and sims keep working.
- Media filenames / old demo video — optional; leave as historical artifacts or move under `media/archive/`.
- Isaac Sim / robot USD names — rename only when you touch the sim asset.

### 2.5 Suggested rename checklist PR title

`chore: rename f1tenth_* packages to porter_*`

Keep the diff mechanical. Behavior changes land in follow-up PRs.

---

## 3. High-level plan (after rename)

Phases are ordered so each one produces something you can drive or measure. Later phases assume earlier ones are “good enough,” not perfect.

### Phase 0 — Concept lock (mechanical)

**Goal:** Prove the Slash + wagon idea is physically viable before deep software.

- Hitch / attachment mockup (tow bar, pin, or push plate).
- Teleop pull tests: empty wagon → typical grocery load.
- Record: starts on smooth concrete, uphill segments, tight turns, jackknife, tip risk, garage clearance.
- Go / no-go: stay with tow, switch to push-behind, or drive-under wagon redesign.

**Exit criteria:** Chosen platform geometry + max payload + safe teleop speed.

### Phase 1 — Bring-up & safety baseline

**Goal:** Reliable manual operation with a kill switch.

- Keep / refresh `porter_drive` twist→ackermann path.
- E-stop (hardware + software topic).
- Battery, wiring, basic teleop launch (`porter_bringup`).
- Log bags for every test run.

**Exit criteria:** You can teleop garage → door slowly with load, and stop instantly.

### Phase 2 — Teach-and-repeat path following

**Goal:** Autonomy on a known route without full SLAM yet.

- Reuse bag→CSV path extraction (`utils/extract_path_from_rosbag.py`).
- Retune existing MPC for **slow**, stoppable motion (lower reference velocity; stronger accel / steering-rate weights).
- Optional: trailer hitch angle sensor or conservative footprint inflation.
- RViz markers for path + predicted trajectory.

**Exit criteria:** Autonomous repeat of a recorded garage→door path in a clear environment (human supervising).

### Phase 3 — State estimation

**Goal:** Pose that does not drift enough to miss doorways.

- Wheel odometry + IMU fusion (e.g. `robot_localization`).
- Calibrate wheelbase / steering limits for Slash under load.
- Characterize indoor drift; decide when lidar/camera localization is required.

**Exit criteria:** Closed-loop runs stay on path for a full mission with bounded lateral error.

### Phase 4 — Mapping & localization (SLAM)

**Goal:** Relocalize on a stored map of the route.

- 2D lidar (or depth) map of garage + corridor + landing.
- Save map; localize on startup (no need to rebuild every run).
- Handle garage lighting changes and feature-sparse halls.

**Exit criteria:** Start anywhere near the car, localize, then execute the mission path.

### Phase 5 — Local planning & people safety

**Goal:** Not just track a line — react to the world.

- Costmap + local planner (or MPC with obstacle constraints).
- Keep-out zones (stairs, elevator shaft, car body).
- Behavior states: stow / wait for human / resume / abort to safe pose.

**Exit criteria:** Completes mission with mild obstacles and a person crossing the path.

### Phase 6 — Mission executive

**Goal:** One button: “bring groceries upstairs.”

- State machine: dock at car → load wait → navigate → arrive → wait for unload.
- Simple UI (phone, laptop, or handheld button).
- Health monitoring: battery, localization covariance, e-stop, trailer angle.

**Exit criteria:** Supervised full mission from car to apartment door without joystick.

### Phase 7 — Learning / ML (optional stretch)

**Goal:** Robustness where classical stack is brittle.

- Perception: door, person, wagon load present/absent.
- Scene classification (garage vs outdoor vs hallway).
- Only then consider learned local policies for ugly segments (thresholds, glare, clutter).

**Exit criteria:** Measurable win on failure cases from Phases 4–6, not ML for its own sake.

---

## 4. Package layout to grow into

Target overlay (add over time; do not create empty shells on day one):

```text
src/
  porter_bringup/      # launch + params for real robot
  porter_drive/        # twist↔ackermann, base interface
  porter_control/      # path MPC / tracking controllers
  porter_localize/     # EKF, lidar localize wrappers
  porter_nav/          # maps, planners, behavior tree / FSM
  porter_perception/   # cameras, detectors (later)
  porter_description/  # URDF: Slash + hitch + wagon footprint
utils/                 # bag path extraction, calibration scripts
maps/                  # saved route maps
missions/              # recorded paths / mission YAML
```

---

## 5. Near-term next actions

1. Hitch pull-test (Phase 0) — before large software investment.
2. Pick final name (**Porter** recommended) and land the rename PR (Section 2).
3. Retune MPC for low-speed path tracking on a recorded home route (Phase 2).
4. Add `porter_description` URDF including wagon footprint once hitch geometry is known.

---

## 6. Success definition (v1)

A supervised robot that, after you load the wagon at the car, drives the groceries from garage to apartment door without a joystick, stops for people/obstacles, and waits for unload — on the Traxxas Slash platform you already have.
