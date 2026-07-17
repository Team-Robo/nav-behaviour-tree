# nav-behaviour-tree

A BehaviorTree.CPP v3 + Nav2 navigation stack for the Galaxea R1 robot,
using ROS 2 Humble.

---

## I. Requirements

| Category          | Requirement                                               |
| ----------------- | --------------------------------------------------------- |
| OS                | Ubuntu 22.04 (native or WSL2 on Windows 11)               |
| ROS               | ROS 2 Humble                                              |
| Navigation        | Nav2, `robot_localization`, `slam_toolbox`                |
| Behaviour tree    | `behaviortree_cpp_v3`                                     |
| GPU               | NVIDIA GPU optional (for `rviz2` / `rqt` rendering)       |
| Docker (optional) | Docker Engine + Docker Compose + NVIDIA Container Toolkit |

**Sensor/odometry source:** a simulation or the real robot must publish
`/livox/lidar` (Livox Mid-360 `PointCloud2`), `/hdas/imu_chassis`
(`hdas_msg/Imu`), and `/odom` (wheel odometry). Without these topics the stack
launches but has no sensor input.

### ROS / Apt Packages

Installed automatically by Docker or `rosdep` (see below):

- `ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-nav2-msgs`
- `ros-humble-robot-localization`
- `ros-humble-slam-toolbox`
- `ros-humble-pointcloud-to-laserscan`
- `ros-humble-behaviortree-cpp-v3`
- `ros-humble-teleop-twist-keyboard`
- `ros-humble-foxglove-bridge`
- `ros-humble-rviz2`, `ros-humble-rqt`, `ros-humble-rqt-common-plugins`
- `ros-humble-tf2-tools`, `ros-humble-rqt-tf-tree`

---

## II. Setup & Installation

### Option A: Docker

**Prerequisites:** Docker, Docker Compose, NVIDIA Container Toolkit (optional), WSLg (if on Windows).

```bash
# Clone the repository
git clone https://github.com/Team-Robo/nav-behaviour-tree.git ~/nav-behaviour-tree
cd ~/nav-behaviour-tree

# Allow GUI forwarding (run once per reboot)
xhost +local:root

# Build and start the container
docker compose build
docker compose up -d

# Enter the container
docker exec -it nav-behaviour-tree bash

# Inside the container — build the workspace
cd /ros2_ws
colcon build
source install/setup.bash
```

#### To stop the container

```bash
docker compose down
```

**Workspace mounting:** `./ros2_ws/src` is bind-mounted to `/ros2_ws/src` inside the container. Edits from your host editor are reflected instantly.

**No NVIDIA GPU?** The GPU passthrough lines in `docker-compose.yml` (the two `NVIDIA_*` environment variables and the `deploy` block) are commented out by default, so `docker compose build`/`up` works as-is on any machine — `rviz2`/`rqt` just fall back to software (llvmpipe) rendering. If you do have an NVIDIA GPU + the NVIDIA Container Toolkit installed, uncomment those lines for hardware-accelerated rendering.

### Option B: Native Ubuntu

**Prerequisites:** Ubuntu 22.04 with ROS 2 Humble installed
([instructions](https://docs.ros.org/en/humble/Installation.html)).

```bash
git clone https://github.com/Team-Robo/nav-behaviour-tree.git ~/nav-behaviour-tree
cd ~/nav-behaviour-tree

source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2_ws/src --ignore-src -r -y   # install dependencies

cd ros2_ws
colcon build
source install/setup.bash
```

After setup, add to your `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/nav-behaviour-tree/ros2_ws/install/setup.bash
```

---

## III. How to Run

All commands assume the workspace is built and sourced (see above). Inside the
container if using Docker.

### Navigate (full bringup)

```bash
ros2 launch nav_behaviour_tree bringup_launch.py
```

### Build a map (SLAM)

```bash
ros2 launch nav_behaviour_tree slam_launch.py
```

Then drive manually in a second terminal while `slam_toolbox` builds the map:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Once the map looks complete, save it (writes `<name>.yaml` + `<name>.pgm`):

```bash
ros2 run nav2_map_server map_saver_cli -f ros2_ws/src/nav_behaviour_tree/maps/<map_name>
```

---

## IV. Repository Structure

```
nav-behaviour-tree/
├── Dockerfile, docker-compose.yml          # ROS Humble dev container
└── ros2_ws/src/
    ├── hdas_msg/                           # vendored R1 IMU/driver message defs
    └── nav_behaviour_tree/
        ├── src/bt_main.cpp                 # the tree executor
        ├── include/.../nav_to_pose.hpp     # the one leaf
        ├── bt_xml/main_tree.xml            # the one tree
        ├── launch/
        │   ├── bringup_launch.py           # Nav2 + tree + rviz + fusion
        │   └── slam_launch.py              # mapping mode
        ├── config/                         # Nav2, EKF, AMCL, SLAM params
        ├── scripts/imu_hdas_translator.py  # HDAS IMU -> sensor_msgs/Imu
        ├── maps/apartment.{yaml,pgm}       # pre-built test map
        └── rviz/
            ├── nav_behaviour_tree.rviz     # navigation view (map + costmaps + pose)
            └── slam.rviz                   # mapping view (live map + scan)
```

---

## V. Technical Explanation

Data flows through two chains that meet at the TF tree.

### Localization chain

Runs in both bringup and SLAM. Raw sensor data is normalized and fused into the
robot's pose:

- `pointcloud_to_laserscan` flattens the Livox `PointCloud2` into a 2D
  `LaserScan` (`/scan`), because neither the sim nor the real driver emits a
  native `LaserScan` and AMCL/costmaps need one.
- `imu_hdas_translator.py` converts the robot's `hdas_msg/Imu` into a standard
  `sensor_msgs/Imu`, correcting the IMU mount's ~180° roll and relabeling its
  `frame_id` so it resolves in the TF tree.
- `ekf_node` (`robot_localization`) fuses `/odom` (velocity) + `/imu` (heading)
  into the `odom -> base_link` transform. AMCL separately provides `map -> odom`
  by matching `/scan` against the map. Together they give a continuous
  `map -> base_link`.

### Navigation chain

Runs in bringup only. `bt_main` builds the behaviour tree and ticks it; the
`NavToPose` leaf wraps Nav2's `navigate_to_pose` action (sends a goal, polls
without blocking, cancels on halt). Nav2's planner (`SmacPlannerHybrid`) and
controller (`RotationShim` + DWB) then drive the base along the planned path,
using the localization chain's pose and the costmaps built from `/scan`.

### How the tree is built, extended, and run

The tree is defined in XML and assembled at startup by `bt_main.cpp`:

1. **Leaves** are C++ classes (e.g. `NavToPose` in
   `include/nav_behaviour_tree/`), each bound to an XML tag string. `bt_main.cpp`
   registers them with the `BT::BehaviorTreeFactory` in block **(A)** —
   `registerRosLeaf<T>(...)` for leaves that need the ROS node (to call
   actions/services), or `factory.registerNodeType<T>(...)` for pure
   synchronous leaves that don't.
2. **Trees** live as `.xml` files under `bt_xml/`, referencing those tags.
   `main_tree.xml` is a `Sequence` of two `NavToPose` calls (a 2-point patrol).
   Each XML file is loaded in block **(B)** via
   `registerBehaviorTreeFromFile(...)`.
3. **At runtime**, `bt_main` builds the tree named by the `tree_id` parameter
   (block **(C)**, default `MainTree`) and ticks its root at `tick_hz`
   (default 20 Hz) until the tree returns SUCCESS or FAILURE.

To add a new behaviour:

1. Write `include/nav_behaviour_tree/my_leaf.hpp`, modeled on `nav_to_pose.hpp`
   (`BT::StatefulActionNode` for async/action work, `BT::SyncActionNode` /
   `BT::ConditionNode` for instant checks).
2. `#include` it in `bt_main.cpp` and register it in block **(A)**.
3. Reference its tag from `main_tree.xml`, or add a new `bt_xml/*.xml` and
   register that file in block **(B)**.
4. Rebuild (`colcon build`) and run it — the default tree via
   `bringup_launch.py`, or a specific one by ID:

```bash
ros2 run nav_behaviour_tree bt_main --ros-args -p tree_id:=MyTree
```

---

## VI. References

- [BehaviorTree.CPP v3](https://www.behaviortree.dev/)
- [Nav2](https://docs.nav2.org/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
