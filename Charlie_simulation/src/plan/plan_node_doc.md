# Plan Node Documentation

The **Plan Node** is the strategic brain of the Charlie robot. It uses a **Behavior Tree (BT)** for real-time decision making based on sensor data and coordinates the main mission: find and activate a red valve.

---

## 1. Architecture Overview

The node bridges perception (`sense`) and action (`act`).

### ROS2 Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/clock` | `Clock` | Startup synchronization with Gazebo |
| `/sense/proximity/front` | `Range` | Front ultrasonic sensor (center) |
| `/sense/proximity/front_left` | `Range` | Front-left ultrasonic sensor |
| `/sense/proximity/front_right` | `Range` | Front-right ultrasonic sensor |
| `/sense/odometry` | `Pose2D` | Robot position (`x`, `y`, `theta`) |
| `/sense/detection` | `String` (JSON) | Visual detections from camera |
| `/sense/detection_zone` | `String` | Color detection zone (`left`, `center`, `right`) |
| `/sense/battery` | `Float32` | Battery level (0-100%) |

### ROS2 Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/plan/command` | `String` | Commands for Act (`Front`, `Left`, `Right`, `Stop`, `Back`, etc.) |
| `/plan/signals` | `String` (JSON) | Event signals (`PersonFound`, `ValveActivated`) |

---

## 2. Startup Sequence

1. Subscribes to `/clock` (published by Gazebo)
2. When `clock.sec > 0`, simulation is active
3. **Waits 15 seconds** for robot stabilization
4. Starts Behavior Tree tick at 10Hz

---

## 3. Target Configuration

Targets are defined in `behaviors.py`:

```python
KNOWN_TARGETS = {
    'green': {'x': -3.2, 'y': -5.5, 'theta': -1.57},
    'blue': {'x': 0.0, 'y': -4.0, 'theta': 0.0},
    'red': {'x': -6.25, 'y': -2.0, 'theta': 3.14},  # Valve
}
```

The robot does **not** know which target is the valve - it must visit each one.

---

## 4. Behavior Tree Structure

### Main Sequence

1. **InitialRetreat** (~1m): Backs away from spawn platform using odometry
2. **Battery Management**: If battery < 20%, returns to charge
3. **Target Loop** (repeats until valve found):
   - `CalculateTarget`: Selects closest unvisited target
   - `GoToTarget`: Navigates with obstacle avoidance
   - `RecognitionValve`: Checks if target is the red valve
4. **ActiveValve**: Activates valve, mission complete
5. **GoHome**: Returns to spawn platform

---

## 5. Key Behaviors

### A. Target Selection (`CalculateTarget`)

- Uses odometry with correction offset
- Selects **closest unvisited target** by Euclidean distance
- Maintains current target until marked as visited
- Returns `FAILURE` when all targets visited

### B. Navigation (`MoveToTarget`)

**Obstacle Avoidance** (ultrasonic only):
- Front sensor < 0.5m → avoid
- Side sensors < 0.3m → avoid
- Uses `calculate_best_direction()` for smart avoidance

**Navigation** (hysteresis-based):
- Well aligned (< 11°) → `MOVE_FORWARD`
- Large error (> 23°) → `MOVE_FRONT_LEFT/RIGHT`
- Medium error → maintain previous action

### C. Target Approach (`AtTarget`)

**Phase 1: Color Detection & Centering** (< 1m from target)
- Color detected but not centered → `MOVE_FRONT_LEFT/RIGHT` (gentle correction)
- Color centered → `MOVE_FORWARD` (straight approach)

**Phase 2: Proximity Detection**
- Front sensor < 0.15m → target reached
- Mark target as visited
- Check if color is red (valve)

> **Note**: Uses proximity sensors instead of bumper to avoid physical collision and odometry drift.

---

## 6. Command Translation

```python
ACTION_TO_COMMAND = {
    'MOVE_FORWARD': 'Front',
    'MOVE_BACKWARD': 'Back',
    'TURN_LEFT': 'Left',
    'TURN_RIGHT': 'Right',
    'MOVE_FRONT_LEFT': 'FrontLeft',
    'MOVE_FRONT_RIGHT': 'FrontRight',
    'STOP': 'Stop',
}
```

---

## 7. Logging Format

All logs use bracketed prefixes without emojis:

```
[PLAN] HOME SAVED @ (0.00, 0.00)
[PLAN] RETREAT START (1.0m)...
[PLAN] RETREAT: 50% (0.50m)
[PLAN] RETREAT COMPLETE (1.02m) - Navigation started
[PLAN] NEW TARGET: BLUE @ (0.00, -4.00) | Dist: 4.02m
[PLAN] NAV -> BLUE | Dist: 3.50m | Ang: -15deg | MOVE_FORWARD
[PLAN] COLOR DETECTED: BLUE at BLUE (dist: 0.85m)
[PLAN] COLOR CENTERED - advancing to BLUE...
[PLAN] TARGET REACHED: BLUE (sensor: 0.12m) | Color: blue
[AVOID] ULTRASONIC: C=0.42<0.5 -> TURN_LEFT
[AVOID] OBSTACLE CLEARED - recovery (1/3)
```

---

## 8. Files

- [behaviors.py](file:///home/salinux/Robotica/Progetto/Robotica_25/src/plan/behaviors.py) - BT node definitions
- [plan_node.py](file:///home/salinux/Robotica/Progetto/Robotica_25/src/plan/plan_node.py) - ROS2 node
