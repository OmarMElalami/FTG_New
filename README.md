# FTG_New — Follow The Gap with LiDAR

Autonomous racing stack for the F1TENTH / MXCK platform. The system connects to a
car's LiDAR over SSH, streams `/scan` data onto the ROS 2 network, converts every
measurement into a circle obstacle, and feeds the result to a Follow-The-Gap planner.

## Architecture

```
Car (192.168.0.100)          Development PC (Docker / ROS 2)
┌──────────────┐            ┌──────────────────────────────────────┐
│ LiDAR + ROS2 │ ──/scan──▶ │ obstacle_substitution                │
│ (mxck_run)   │            │   /scan → /obstacles                 │
└──────────────┘            │                                      │
      ▲                     │ Follow-The-Gap Planner               │
      │ SSH                 │   /obstacles → steering commands     │
      │                     └──────────────────────────────────────┘
lidar_vs (Start_Lidar.py)
```

## Packages

| Package | Description |
|---------|-------------|
| `lidar_vs` | SSH-connects to the car and starts the LiDAR. Provides the combined launch file. |
| `obstacle_substitution` | Subscribes to `/scan` (LaserScan) and publishes `/obstacles` (ObstaclesStamped). |

## Prerequisites

- ROS 2 (tested on Humble)
- `sshpass` for password-based SSH, **or** SSH key-based auth configured for the car
- `obstacle_msgs` package built and sourced

## Build

```bash
colcon build --symlink-install \
  --packages-select lidar_vs obstacle_substitution \
  --allow-overriding obstacle_substitution
```

## Run

```bash
ros2 launch lidar_vs lidar_with_obstacles_launch.py
```

This single command:
1. SSH-connects to the car and starts the LiDAR (publishes `/scan`)
2. Launches `obstacle_substitution` after a 3-second delay (publishes `/obstacles`)
