# wrist_games_ros2

A ROS2 Python package of wrist-controlled mini-games for rehabilitation and motor training. A single persistent **Game Manager** node exposes one `Trigger` service per game. Calling a service launches the selected game as a subprocess, which subscribes to a `sensor_msgs/JointState` topic and maps three wrist degrees of freedom to in-game controls.

---

## Table of contents

1. [Package structure](#package-structure)
2. [Joint layout](#joint-layout)
3. [Games](#games)
4. [Architecture](#architecture)
5. [Build & install](#build--install)
6. [Running via launch files (recommended)](#running-via-launch-files-recommended)
7. [Running via `ros2 run`](#running-via-ros2-run)
8. [Launching games via service calls](#launching-games-via-service-calls)
9. [Running a game standalone](#running-a-game-standalone)
10. [ROS2 parameters reference](#ros2-parameters-reference)
11. [Sound assets](#sound-assets)

---

## Package structure

```
wrist_games_ros2/
├── launch/
│   ├── game_manager.launch.py   ← PRIMARY launch file (starts the manager node)
│   ├── airplane_game.launch.py  ┐
│   ├── catcher_game.launch.py   │
│   ├── jedi_game.launch.py      │ standalone per-game launch files
│   ├── octagon_game.launch.py   │
│   ├── pendulum_game.launch.py  │
│   ├── tunnel_game.launch.py    │
│   └── xwing_game.launch.py    ┘
├── wrist_games_ros2/
│   ├── game_manager.py          ← ROS2 node; owns all Trigger services
│   ├── joint_state_bridge.py    ← subscribes to JointState, exposes get_normalized()
│   ├── score_sound.py           ← ScoreBoard, LevelManager, SoundManager helpers
│   ├── airplane_game.py
│   ├── catcher_game.py
│   ├── jedi_game.py
│   ├── octagon_game.py
│   ├── pendulum_game.py
│   ├── tunnel_game.py
│   ├── xwing_game.py
│   └── assets/                  ← optional .wav files (start, score, miss, level_up)
├── resource/wrist_games_ros2    ← ament resource marker
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Joint layout

All games read from a `sensor_msgs/JointState` topic. The package expects **exactly three joints**, mapped by positional index (or by name — see [parameters](#ros2-parameters-reference)):

| Index | Physical DoF | Convention name | Used for |
|-------|-------------|-----------------|----------|
| **0** | Pronation / Supination (PS) | **turn** | Shape rotation (tunnel game) |
| **1** | Flexion / Extension (FE)   | **y**    | Vertical movement in all 2-D games |
| **2** | Radial / Ulnar Deviation (RUD) | **x** | Horizontal movement in all 2-D games |

Raw joint positions are passed through `tanh(position × gain)` to produce a normalised value in `[-1, 1]`. The `control_gain` parameter scales sensitivity before the `tanh` clamp.

> **Using named joints?**  Set `joint_names:=ps,fe,rud` (or whatever your device publishes) via the launch argument or `--joint-names` CLI flag to look up joints by name instead of index.

---

## Games

| Service | Game | DoF used | Description |
|---------|------|----------|-------------|
| `/wrist_games/start_airplane` | **Airplane** | y (FE) | Fly a plane through scrolling gate pairs without hitting the pillars. Speed and gap size tighten each level. |
| `/wrist_games/start_catcher`  | **Wrist Catcher** | x (RUD) | Move a horizontal paddle to catch falling balls. Paddle shrinks and ball speeds up each level. |
| `/wrist_games/start_jedi`     | **Jedi Deflect** | y (FE) + x (RUD) | Tilt wrist to rotate a lightsaber and deflect incoming blaster bolts from all directions. |
| `/wrist_games/start_octagon`  | **Octagon Squeeze** | y (FE) + x (RUD) | Keep a cursor inside a slowly shrinking, spinning octagon. Score every second while inside. |
| `/wrist_games/start_pendulum` | **Balance Pendulum** | x (RUD) | Classic inverted pendulum — move the cart to keep the pole upright against random disturbances. |
| `/wrist_games/start_tunnel`   | **Shape Tunnel** | turn (PS) + y (FE) + x (RUD) | Fit a shape through approaching walls by aligning its position *and* rotation using all three wrist axes. |
| `/wrist_games/start_xwing`    | **X-Wing** | y (FE) + x (RUD) | Pilot an X-Wing through space, destroying TIE fighters with auto-firing lasers. |
| `/wrist_games/stop_game`      | *(any)*    | —        | Terminate whatever game is currently running. |

### Scoring and progression

- Every game uses a shared 1-based level system. Level increases every **50 points** (configurable via `points_per_catch`).
- On each level-up the game plays a sound and increases difficulty (speed, shrinkage rate, tighter tolerances, etc.).
- **Catching / scoring** awards `points_per_catch` points and resets on a miss.
- **Lives** reset to `start_lives` when exhausted (the game keeps running for continuous therapy use).

### Per-game controls

| Game | y joint (FE) | x joint (RUD) | turn joint (PS) | Notes |
|------|-------------|--------------|----------------|-------|
| Airplane | pitch (up/down) | — | — | Only 1 DoF |
| Catcher | — | paddle position | — | Only 1 DoF |
| Jedi Deflect | saber vertical angle | saber horizontal angle | — | atan2 of both axes |
| Octagon | cursor Y | cursor X | — | |
| Pendulum | — | cart X position | — | Only 1 DoF |
| Shape Tunnel | shape Y position | shape X nudge | shape rotation | Full 3-DOF |
| X-Wing | ship Y position | ship X position | — | |

---

## Architecture

```
                  ros2 service call  (or Web GUI via rosbridge)
                          │
                          ▼
              ┌───────────────────────┐
              │   GameManagerNode     │   wrist_game_manager
              │   game_manager.py     │
              │                       │
              │  /wrist_games/start_* │◄── Trigger services (one per game)
              │  /wrist_games/stop_*  │
              └──────────┬────────────┘
                         │ subprocess.Popen
                         │ [python -m wrist_games_ros2.<game> --ros-topic ...]
                         ▼
              ┌───────────────────────┐
              │   Game Process        │   (separate OS process)
              │   e.g. xwing_game.py  │
              │                       │
              │ rclpy.init()          │
              │ JointStateBridge(Node)│◄── subscribes to /joint_states
              │ pygame event loop     │
              └───────────────────────┘
```

**Key design decisions:**

- The game process runs **pygame in its own OS process**, so the blocking render loop never starves the ROS2 executor in `GameManagerNode`.
- `JointStateBridge` is a minimal `rclpy.Node` that subscribes to `JointState`, caches the latest three positions behind a lock, and exposes `get_normalized(gain)`.
- Only **one game can run at a time**. Calling a start service while another game is running returns `success=False` with an explanatory message.

---

## Build & install

```bash
# 1. Place the package inside your ROS2 workspace source directory
cd ~/ros2_ws/src
# (copy or git clone wrist_games_ros2 here)

# 2. Install Python dependencies
pip install pygame>=2.5.0

# 3. Build
cd ~/ros2_ws
colcon build --packages-select wrist_games_ros2
source install/setup.bash
```

---

## Running via launch files (recommended)

### Start the Game Manager

```bash
ros2 launch wrist_games_ros2 game_manager.launch.py
```

With overrides:

```bash
ros2 launch wrist_games_ros2 game_manager.launch.py \
    ros_topic:=/openwrist/joint_states \
    joint_names:=ps,fe,rud \
    control_gain:=1.5 \
    start_lives:=5
```

### Start a game directly (standalone, for testing)

```bash
ros2 launch wrist_games_ros2 tunnel_game.launch.py
ros2 launch wrist_games_ros2 tunnel_game.launch.py ros_topic:=/wrist/joint_states control_gain:=2.0
```

Each individual launch file accepts the same arguments as the game manager (`ros_topic`, `joint_names`, `control_gain`, `start_lives`, `points_per_catch`) plus the joint-index overrides relevant to that game.

---

## Running via `ros2 run`

```bash
# Start manager with default settings
ros2 run wrist_games_ros2 wrist-game-manager

# Start manager with custom ROS2 parameters
ros2 run wrist_games_ros2 wrist-game-manager \
  --ros-args \
  -p ros_topic:=/openwrist/joint_states \
  -p joint_names:=ps,fe,rud \
  -p joint_yaw_index:=0 \
  -p joint_v_index:=1 \
  -p joint_h_index:=2 \
  -p control_joint_index:=2 \
  -p control_gain:=1.5 \
  -p start_lives:=3 \
  -p points_per_catch:=10
```

---

## Launching games via service calls

Once the manager is running:

```bash
# Start a game
ros2 service call /wrist_games/start_xwing     std_srvs/srv/Trigger {}
ros2 service call /wrist_games/start_tunnel    std_srvs/srv/Trigger {}
ros2 service call /wrist_games/start_airplane  std_srvs/srv/Trigger {}
ros2 service call /wrist_games/start_catcher   std_srvs/srv/Trigger {}
ros2 service call /wrist_games/start_jedi      std_srvs/srv/Trigger {}
ros2 service call /wrist_games/start_octagon   std_srvs/srv/Trigger {}
ros2 service call /wrist_games/start_pendulum  std_srvs/srv/Trigger {}

# Stop current game
ros2 service call /wrist_games/stop_game std_srvs/srv/Trigger {}
```

From a **web GUI** via `rosbridge_suite` + `roslibjs`:

```js
const svc = new ROSLIB.Service({
  ros,
  name: '/wrist_games/start_tunnel',
  serviceType: 'std_srvs/srv/Trigger',
});
svc.callService(new ROSLIB.ServiceRequest({}), res => {
  console.log(res.success, res.message);
});
```

---

## Running a game standalone

Every game can also be run directly without the manager, useful for quick testing:

```bash
# via ros2 run
ros2 run wrist_games_ros2 wrist-tunnel-game \
  --ros-topic /joint_states --joint-yaw-index 0 --joint-v-index 1 --joint-h-index 2

# via python -m
python -m wrist_games_ros2.tunnel_game --ros-topic /joint_states

# available executables:
#   wrist-game-manager
#   wrist-airplane-game
#   wrist-catcher-game
#   wrist-jedi-game
#   wrist-octagon-game
#   wrist-pendulum-game
#   wrist-tunnel-game
#   wrist-xwing-game
```

All game executables accept `--help` for the full argument list.

---

## ROS2 parameters reference

These parameters are declared on the **Game Manager node** and forwarded to every game subprocess:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ros_topic` | `string` | `/joint_states` | `JointState` topic the device publishes to |
| `joint_names` | `string` | `""` | Comma-separated joint names to look up by name instead of index (e.g. `ps,fe,rud`). Empty = use indices. |
| `joint_yaw_index` | `int` | `0` | Index of the **turn / PS** joint in `position[]` |
| `joint_v_index` | `int` | `1` | Index of the **y / FE** joint in `position[]` |
| `joint_h_index` | `int` | `2` | Index of the **x / RUD** joint in `position[]` |
| `control_joint_index` | `int` | `2` | Joint index for 1-DoF games (catcher paddle). Defaults to x joint. |
| `control_gain` | `double` | `1.0` | Multiplier applied before `tanh` normalisation. Higher = more sensitive. |
| `start_lives` | `int` | `3` | Lives each game session starts with |
| `points_per_catch` | `int` | `10` | Score awarded per successful action; also controls level-up threshold (50 pts = level 2, 100 pts = level 3, …) |

---

## Sound assets

Place WAV files in `wrist_games_ros2/assets/` to enable audio feedback:

| File | Triggered when |
|------|---------------|
| `start.wav` | Game window opens |
| `score.wav` | Successful action (gate passed, ball caught, bolt deflected, …) |
| `miss.wav` | Life lost |
| `level_up.wav` | Level threshold crossed |

If WAV files are absent, Windows system beeps are used as a fallback. To disable all sound:

```bash
export WRIST_GAMES_SOUND=0
```
