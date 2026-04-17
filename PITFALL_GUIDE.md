# AUT4508 Group Project Pitfall Guide

> Based on all real issues encountered during Part 1 development on WSL2.
> Every entry is a real bug that cost debugging time. Read before starting any Part.

---

## 1. Gazebo Harmonic on WSL2

### 1.1 Robot Invisible in GUI (Most Critical)

**Symptom:** `ros2 launch` succeeds, topics (`/odom`, `/scan`, `/imu`) publish normally, `gz model --list` shows the robot, but **nothing appears** in the Gazebo GUI window.

**Root Cause:** `ros_gz_sim create` (dynamic spawn) adds the model to the physics server, but the GUI render process on WSL2 does not pick up the new entity. This is a Gazebo Harmonic + WSL2 rendering sync bug.

**Fix:** Never use dynamic spawn. Include the robot directly in the world SDF:

```xml
<!-- In your world .sdf file, before </world> -->
<include>
  <uri>model://pioneer</uri>
  <pose>0 0 0.02 0 0 0</pose>
  <name>pioneer</name>
</include>
```

**Requirements for `model://` to work:**
1. Model directory structure must be:
   ```
   models/pioneer/
     model.config    <-- REQUIRED, Gazebo will error without this
     model.sdf
   ```
2. `model.config` minimal content:
   ```xml
   <?xml version="1.0"?>
   <model>
     <name>pioneer</name>
     <version>1.0</version>
     <sdf version="1.8">model.sdf</sdf>
     <description>Pioneer 3-AT robot</description>
   </model>
   ```
3. `GZ_SIM_RESOURCE_PATH` must include the **parent** of the `pioneer/` folder (i.e., the `models/` directory), not the `pioneer/` folder itself.

**In launch file:**
```python
models_path = os.path.join(description_share, "models")
resource_path = ":".join([
    models_path,
    os.path.dirname(description_share),
    os.path.dirname(bringup_share),
])
SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_path)
```

**Verification:** After launching, the robot should be visible immediately when the Gazebo GUI opens. No spawn delay needed.

### 1.2 GUI Camera Shows Entire Field, Robot Looks Like a Pixel

**Symptom:** Gazebo opens, world is visible, but camera is zoomed way out. The 0.5m robot is invisible on a 32x24m field.

**Root Cause:** Old-format SDF `<gui><camera>` section is silently ignored by Gazebo Harmonic. Harmonic requires the MinimalScene plugin format.

**Wrong (Gazebo Classic / Fortress):**
```xml
<gui>
  <camera name="user_camera">
    <pose>-2 -6 4 0 0.6 1.3</pose>
  </camera>
</gui>
```

**Correct (Gazebo Harmonic — performance-optimized for WSL2):**
```xml
<gui fullscreen="0">
  <!-- REQUIRED: 3D rendering -->
  <plugin filename="MinimalScene" name="3D View">
    <gz-gui>
      <title>3D View</title>
      <property type="bool" key="showTitleBar">false</property>
      <property type="string" key="state">docked</property>
    </gz-gui>
    <engine>ogre2</engine>
    <scene>scene</scene>
    <ambient_light>0.4 0.4 0.4</ambient_light>
    <background_color>0.8 0.8 0.8</background_color>
    <camera_pose>-1 -2 1.5 0 0.35 1.2</camera_pose>
    <camera_clip>
      <near>0.25</near>
      <far>500</far>
    </camera_clip>
  </plugin>
  <!-- REQUIRED: scene graph management -->
  <plugin filename="GzSceneManager" name="Scene Manager">
    <gz-gui>
      <property type="bool" key="showTitleBar">false</property>
      <property type="string" key="state">docked</property>
    </gz-gui>
  </plugin>
  <!-- REQUIRED: mouse interaction (rotate/pan/zoom) -->
  <plugin filename="InteractiveViewControl" name="Interactive view control">
    <gz-gui>
      <property type="bool" key="showTitleBar">false</property>
      <property type="string" key="state">docked</property>
    </gz-gui>
  </plugin>
  <!-- REQUIRED: play/pause/step controls -->
  <plugin filename="WorldControl" name="World control">
    <gz-gui>
      <title>World control</title>
      <property type="bool" key="showTitleBar">false</property>
      <property type="string" key="state">docked</property>
    </gz-gui>
    <play_pause>true</play_pause>
    <step>true</step>
    <start_paused>false</start_paused>
  </plugin>
  <!--
    REMOVED for WSL2 performance (each adds GPU/CPU overhead):
    - CameraTracking: camera-follow feature, not needed with mouse control
    - WorldStats: real-time factor / sim time display
    - EntityTree: model/link hierarchy browser
    Add them back only if specifically needed for debugging.
  -->
</gui>
```

**`camera_pose` format:** `x y z roll pitch yaw` (meters, radians). Set it close to the robot spawn point.

**Performance tip:** On WSL2 with integrated GPU (e.g., Radeon 780M), every GUI plugin adds overhead to the ogre2→OpenGL→Mesa d3d12→DX12 pipeline. Strip to the 4 essential plugins above for smooth rendering.

### 1.3 Stale Gazebo Processes

**Symptom:** Launching again after Ctrl+C gives errors, topics from previous run still active, or Gazebo refuses to start.

**Root Cause:** `gz sim server`, `gz sim gui`, bridge, sanitizer, and Nav2 processes sometimes survive `Ctrl+C`, especially on WSL2.

**Fix:** Always kill stale processes before relaunching:
```bash
pkill -f "gz sim" ; pkill -f "ros_gz_bridge" ; pkill -f "robot_state_publisher" ; \
pkill -f "sanitizer" ; pkill -f "slam_toolbox" ; pkill -f "planner_server" ; \
pkill -f "controller_server" ; pkill -f "behavior_server" ; pkill -f "bt_navigator" ; \
pkill -f "waypoint_follower" ; pkill -f "lifecycle_manager" ; \
pkill -f "mission_manager" ; pkill -f "path_recorder"
```

**Best practice:** Add a cleanup command at the beginning of your launch file:
```python
cleanup = ExecuteProcess(
    cmd=["bash", "-lc", "pgrep -af 'gz sim|ros_gz_bridge|...' | awk ... | xargs -r kill -TERM; sleep 1"],
    output="screen",
)
```

### 1.4 OOM on WSL2

**Symptom:** System hangs, processes killed by OOM killer.

**Prevention:**
- Set WSL2 memory limit in `C:\Users\<name>\.wslconfig`:
  ```ini
  [wsl2]
  memory=6GB
  swap=4GB
  ```
- Keep camera resolution low: `160x120` or `320x240`, not `640x480`
- Keep camera update rate low: `2 Hz`, not `30 Hz`
- Don't launch RViz unless needed (`rviz:=false` by default)
- Use `gpu_lidar` (runs on GPU) instead of `ray` sensor type
- Reduce sensor rates: IMU 20Hz (not 50Hz), odom 20Hz (matches controller frequency)
- Increase physics step size: 0.005s instead of 0.004s

### 1.5 Headless Mode (Best WSL2 Performance)

**Problem:** Running Gazebo GUI + RViz simultaneously uses 2 GPU render pipelines through WSLg (ogre2 -> OpenGL -> Mesa d3d12 -> DX12), causing extreme lag.

**Solution:** Run Gazebo in server-only mode (`-s` flag) and use only RViz:
```bash
# Terminal 1 — headless simulation (no GUI)
ros2 launch p3at_bringup sim.launch.py headless:=true

# Terminal 2 — navigation + RViz only
ros2 launch p3at_bringup nav.launch.py rviz:=true
```

**Rule of thumb:** Never open Gazebo GUI and RViz at the same time on WSL2 with integrated graphics.

### 1.6 RViz Camera Not Following Robot

**Symptom:** RViz opens but the camera stays at origin while the robot drives away.

**Root Cause:** RViz `.rviz` config file missing `Views:` section. Defaults to `Orbit` view locked to fixed frame origin.

**Fix:** Add `ThirdPersonFollower` view to the `.rviz` config:
```yaml
  Views:
    Current:
      Class: rviz_default_plugins/ThirdPersonFollower
      Name: ThirdPersonFollower
      Distance: 12
      Pitch: 1.0
      Target Frame: base_link
      Yaw: 0
      Focal Point:
        X: 0
        Y: 0
        Z: 0
```

**RViz mouse controls:**
- Left-click drag = rotate view
- Middle-click drag (or Shift+left) = pan
- Scroll wheel = zoom
- Right-click drag = zoom (vertical)

---

## 2. SDF / URDF Model

### 2.1 URDF vs SDF: You Need Both

| File | Purpose | Used By |
|------|---------|---------|
| `pioneer.urdf.xacro` | TF tree, `robot_state_publisher` | ROS2 side |
| `pioneer/model.sdf` | Physics, sensors, visuals, plugins | Gazebo side |

**They must match** in:
- Link names (exact string match)
- Joint names
- Sensor frame IDs
- Wheel dimensions (`wheel_radius`, `wheel_separation`)

If you change a parameter in one, change it in the other. Mismatches cause TF errors, wrong odometry, or invisible sensors.

### 2.2 Lidar Samples Must Match Real Hardware

**SICK TIM781 spec:** 270 FOV, 0.33/ray = **810 samples**

**Wrong:**
```xml
<samples>360</samples>  <!-- 0.75/ray, too coarse -->
```

**Correct:**
```xml
<horizontal>
  <samples>810</samples>
  <resolution>1</resolution>
  <min_angle>-2.35619</min_angle>   <!-- -135 -->
  <max_angle>2.35619</max_angle>    <!-- +135 -->
</horizontal>
```

This must be set in **both** `model.sdf` and `pioneer.urdf.xacro`.

### 2.3 DiffDrive Plugin: Disable odom TF

**Critical:** The Gazebo DiffDrive plugin can publish both `/odom` topic AND `odom -> base_link` TF. If your architecture uses a separate node for the TF (sanitizer or robot_localization), you **must** disable the plugin's TF:

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  ...
  <publish_odom>true</publish_odom>
  <publish_odom_tf>false</publish_odom_tf>   <!-- CRITICAL -->
</plugin>
```

If both DiffDrive and your node publish `odom -> base_link`, you get TF multi-parent errors and SLAM will fail.

### 2.4 Spawn Height

Don't spawn the robot too high. `z=0.8` causes the robot to freefall and bounce. Use `z=0.02` (just above ground) for minimal settling time.

### 2.5 Sensor gz_frame_id Warnings

```
Warning: XML Element[gz_frame_id], child of element[sensor], not defined in SDF
```

**This is harmless.** `gz_frame_id` is a Gazebo Harmonic extension not in the official SDF spec. The warning appears but the feature works correctly. Ignore it.

### 2.6 Wheel Separation Value

The Pioneer 3-AT has `wheel_separation = 0.394` (meters), not `0.268`. Using `0.268` causes the robot to turn too sharply and odometry to drift. Verify against the real robot's wheel track width.

---

## 3. TF Architecture

### 3.1 The Golden Rule: One Publisher Per TF Frame

```
map -> odom          : slam_toolbox ONLY
odom -> base_link    : odom_sanitizer (or robot_localization) ONLY
base_link -> sensors : robot_state_publisher ONLY
```

**Never** let two nodes publish the same transform. Common violations:
- DiffDrive plugin + odom_sanitizer both publishing `odom -> base_link`
- Multiple robot_state_publisher instances
- EKF + raw odom both publishing transforms

### 3.2 Clock Sanitizer is Essential

**Symptom:** `slam_toolbox` warns "TF extrapolation into the future", SLAM map drifts or doesn't build.

**Root Cause:** Gazebo's `/clock` sometimes has non-monotonic timestamps (time jumps backward briefly). ROS2 nodes using `use_sim_time: true` depend on monotonic clock.

**Fix:** A clock sanitizer node that subscribes to the raw Gazebo clock and republishes only strictly monotonic timestamps:

```python
if new_time > self.last_time:
    self.publisher.publish(msg)
    self.last_time = new_time
```

### 3.3 Odom Sanitizer is Essential

**Problem:** Gazebo's raw `/odom` message has zero covariance matrix and sometimes non-monotonic timestamps. `slam_toolbox` and `robot_localization` both require valid covariance.

**Fix:** An odom sanitizer node that:
1. Fixes timestamps to use the sanitized clock
2. Fills in reasonable covariance values
3. Publishes the `odom -> base_link` TF (since DiffDrive is configured not to)

### 3.4 Joint State Sanitizer

Same timestamp issue as odom. Gazebo's `/joint_states` may have stale timestamps. The sanitizer re-stamps them with current ROS time before `robot_state_publisher` consumes them.

---

## 4. Nav2 Stack

### 4.1 Custom Controller Plugin (C++)

A custom `nav2_core::Controller` plugin is required for Part 1 Task 5. Key architecture:

```
p3at_nav_plugins/
  include/p3at_nav_plugins/p3at_controller.hpp
  src/p3at_controller.cpp
  p3at_nav_plugins.xml        <-- plugin registration
  CMakeLists.txt
```

**Plugin XML must register the class:**
```xml
<class_list>
  <class type="p3at_nav_plugins::P3ATController" base_class_type="nav2_core::Controller">
    <description>P3AT custom controller</description>
  </class>
</class_list>
```

**nav2.yaml must reference it:**
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "p3at_nav_plugins::P3ATController"
```

### 4.2 Collision Check: Don't Treat Unknown as Obstacle

**Symptom:** Robot freezes at map edges or unexplored areas, throws "Predicted collision" continuously.

**Root Cause:** The collision check function treats `NO_INFORMATION` (255) cells as lethal obstacles.

**Wrong:**
```cpp
if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
    cost == nav2_costmap_2d::NO_INFORMATION)  // BUG: treats unknown as wall
```

**Correct:**
```cpp
if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
```

### 4.3 Nav2 Lifecycle Startup Order

Nav2 nodes must be activated in the correct order by `lifecycle_manager`. The `node_names` list determines activation order:

```yaml
node_names:
  - controller_server     # first
  - planner_server
  - behavior_server
  - bt_navigator
  - waypoint_follower     # last
```

If `waypoint_follower` is activated before `bt_navigator`, the action server may not be ready.

### 4.4 Mission Manager: Wait for Map Before Sending Goals

**Symptom:** `FollowWaypoints` goal rejected or planner fails immediately.

**Root Cause:** Sending navigation goals before `slam_toolbox` has produced a map. The planner has no costmap to plan on.

**Fix:** Subscribe to `/map` and only send goals after receiving the first map message, plus an additional delay:

```python
self.create_subscription(OccupancyGrid, "/map", self._map_cb, 10)
# In _maybe_start():
if not self.map_received:
    return
elapsed = (now - self.start_time) / 1e9
if elapsed < self.start_delay_sec:  # e.g., 6 seconds
    return
```

### 4.5 Map Saving After Mission

SLAM maps are lost when nodes shut down. Save them automatically after the mission completes:

```python
subprocess.run([
    "ros2", "run", "nav2_map_server", "map_saver_cli",
    "-f", map_file,
    "--ros-args", "-p", "use_sim_time:=true",
], capture_output=True, text=True, timeout=30)
```

This produces `map_file.pgm` (image) + `map_file.yaml` (metadata).

---

## 5. Build System

### 5.1 colcon build --symlink-install

Always use `--symlink-install` during development. This creates symlinks from `install/` to `src/` so you can edit Python files and config without rebuilding.

```bash
colcon build --symlink-install    # Only needed ONCE (first time)
source install/setup.bash         # Needed in EVERY new terminal
```

**When you MUST rebuild:**
- First time after cloning the repo
- After changing C++ code (e.g., `p3at_controller.cpp`)
- After adding new files (new Python nodes, new configs)
- After changing `CMakeLists.txt` or `setup.py`

**When you do NOT need to rebuild (symlinks handle it):**
- Editing Python scripts (`.py`)
- Editing YAML config files
- Editing launch files (`.launch.py`)
- Editing SDF world/model files

### 5.2 Source Setup: Every Terminal, Not Every Build

```bash
source install/setup.bash
```

**Must be done in every new terminal.** You do NOT need to rebuild before sourcing. Forgetting this is the #1 cause of "package not found" errors.

### 5.3 CMakeLists.txt: Install Directories Recursively

For the description package to install models, urdf, meshes, rviz:
```cmake
install(
  DIRECTORY urdf meshes rviz models
  DESTINATION share/${PROJECT_NAME}
)
```

This installs entire directory trees. If you add new subdirectories (e.g., `models/pioneer/`), this automatically picks them up.

### 5.4 Python Package: entry_points Must List All Executables

In `setup.py`, every Python node must be listed:
```python
entry_points={
    "console_scripts": [
        "clock_sanitizer = p3at_bringup.clock_sanitizer:main",
        "odom_sanitizer = p3at_bringup.odom_sanitizer:main",
        "joint_state_sanitizer = p3at_bringup.joint_state_sanitizer:main",
        "mission_manager = p3at_bringup.mission_manager:main",
        "path_recorder = p3at_bringup.path_recorder:main",
    ],
},
```

Missing an entry here = `ros2 run` / launch file can't find the node.

---

## 6. Bridge Configuration

### 6.1 Topic Name Mapping

Some Gazebo topics need to be remapped to different ROS2 names to avoid conflicts with sanitizer nodes:

```yaml
# bridge.yaml
- ros_topic_name: "/clock_raw"       # NOT /clock -- sanitizer republishes as /clock
  gz_topic_name: "/clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/joint_states_raw" # NOT /joint_states -- sanitizer re-stamps
  gz_topic_name: "/joint_states"
  direction: GZ_TO_ROS
```

If you bridge directly to `/clock` and `/joint_states`, the sanitizer nodes have nothing to fix.

### 6.2 Camera Bridge Type Names

```yaml
- ros_topic_name: "/camera/image"
  gz_topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"       # NOT sensor_msgs/Image
  gz_type_name: "gz.msgs.Image"                # NOT ignition.msgs.Image
  direction: GZ_TO_ROS
```

Gazebo Harmonic uses `gz.msgs.*`, not the old `ignition.msgs.*` naming.

---

## 7. World Design for SLAM

### 7.1 Obstacles Must Have Collision at Lidar Height

**Symptom:** Obstacles appear in Gazebo but the robot drives through them. SLAM doesn't see them.

**Root Cause:** Obstacle collision geometry doesn't intersect the lidar scan plane. If your lidar is at `z=0.281m` and a cone's collision only goes to `z=0.15m`, the lidar beams fly over it.

**Fix:** Make obstacle collision height extend above the lidar plane:
```xml
<collision name="collision">
  <geometry>
    <cylinder>
      <radius>0.15</radius>
      <length>0.5</length>   <!-- Must be > lidar height -->
    </cylinder>
  </geometry>
</collision>
```

### 7.2 World Needs Enough Geometric Features

`slam_toolbox` needs distinct features to build a map. A flat grass field with no obstacles = SLAM has nothing to match against, and the map will drift.

Include: walls/curbs, buildings, light poles, trees, cones, buckets. Spread them around the field so the robot always has features in view.

---

## 8. Part 2 / Part 3 Gotchas (Advance Warning)

### 8.1 Part 2: Sim-to-Real Transition

- **GPS waypoints:** Part 2 uses GPS coordinates, not map-frame coordinates. You'll need `robot_localization` with a `navsat_transform_node` to fuse GPS + IMU + wheel odometry.
- **Real sensors:** Replace Gazebo sensor plugins with actual ROS2 drivers:
  - Lidar: `sick_scan_xd` package
  - Camera: `depthai-ros` (OAK-D)
  - IMU: Custom node preferred over the Phidget ROS2 library (per project spec)
- **Gamepad deadman switch:** Part 2 Task 7 requires Bluetooth gamepad with X = auto mode, O = manual mode, back pedals = deadman switch. Use the `joy` package.
- **Cone photo:** When within 1-2m of a waypoint cone, take a photo and leave the cone on the robot's **right** side.
- **Indoor testing:** Wrap tires with tape for indoor surfaces, or the motors will burn out.

### 8.2 Part 3: Mapping and Discovery

- **Two-phase architecture:** Phase 1 = explore & map 15x15m area; Phase 2 = fast waypoint driving to 3 locations. Must be switchable via button/service, no restart.
- **Greek letter recognition:** Hand-drawn Greek letters on surfaces at knee height. Camera must detect and classify them. OpenCV + a trained classifier or template matching.
- **Color detection:** Find yellow/red obstacles, note their locations.
- **Emergency stop:** Software e-stop if a moving object is within 1m. Save last 5 seconds of data on e-stop event.
- **UI display:** Touch screen must show: map, robot state, photos, planned path.

### 8.3 Shared Architecture Across Parts

Design your Part 1 code with these interfaces so Part 2/3 integration is smooth:
- Keep waypoints in YAML (easy to swap sim waypoints for GPS waypoints)
- Keep sensor processing modular (swap Gazebo bridge for real driver)
- Keep the Nav2 stack (planner + controller + costmaps) -- it works the same on real hardware
- `slam_toolbox` works identically in sim and real, just change the lidar topic if needed

---

## 9. Quick Reference: Launch Commands

```bash
# Build (first time only, or after C++ changes)
cd /home/god/auto4508/project
colcon build --symlink-install
source install/setup.bash   # needed in EVERY new terminal

# Part 1 Demo (optimal for WSL2: Gazebo GUI only, no RViz)
# Terminal 1 - Gazebo + Robot
ros2 launch p3at_bringup sim.launch.py

# Terminal 2 - Navigation + SLAM + Mission (no rviz for max performance)
ros2 launch p3at_bringup nav.launch.py record:=true

# Alternative: headless Gazebo + RViz (no 3D world view)
# Terminal 1
ros2 launch p3at_bringup sim.launch.py headless:=true
# Terminal 2
ros2 launch p3at_bringup nav.launch.py rviz:=true record:=true

# Teleop only (for testing model / Task 1)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Kill everything
pkill -f "gz sim" ; pkill -f "ros_gz_bridge" ; pkill -f "robot_state_publisher" ; \
pkill -f "sanitizer" ; pkill -f "slam_toolbox" ; pkill -f "planner_server" ; \
pkill -f "controller_server" ; pkill -f "behavior_server" ; pkill -f "bt_navigator" ; \
pkill -f "waypoint_follower" ; pkill -f "lifecycle_manager" ; \
pkill -f "mission_manager" ; pkill -f "path_recorder"
```

### Launch Arguments Quick Reference

| Launch File | Argument | Default | Description |
|-------------|----------|---------|-------------|
| `sim.launch.py` | `headless:=true` | `false` | No Gazebo GUI (server only), use RViz instead |
| `sim.launch.py` | `rviz:=true` | `false` | Open RViz (sim view) |
| `sim.launch.py` | `teleop:=true` | `false` | Open keyboard teleop (xterm) |
| `nav.launch.py` | `rviz:=true` | `false` | Open RViz (nav view) — skip on WSL2 if Gazebo GUI is open |
| `nav.launch.py` | `record:=true` | `false` | Record rosbag2 |
| `nav.launch.py` | `autostart:=false` | `true` | Don't auto-send waypoints |

---

## 10. Output Files After Demo

| File | Content | Location |
|------|---------|----------|
| `part1_slam_map.pgm` + `.yaml` | SLAM map (image + metadata) | Working directory |
| `part1_driven_path.csv` | Driven path: timestamp, x, y, z, roll, pitch, yaw | Working directory |
| `part1_demo_run/` | rosbag2 recording (if `record:=true`) | Working directory |

---

## 11. Debugging Cheat Sheet

| Symptom | First Check | Likely Fix |
|---------|-------------|------------|
| Robot not visible in Gazebo | `gz model --list` | Use `<include>` in world SDF, not dynamic spawn |
| No topics publishing | `ros2 topic list` | Check bridge config, restart |
| TF errors / extrapolation | `ros2 run tf2_tools view_frames` | Check for duplicate TF publishers |
| SLAM map not building | `ros2 topic hz /scan` | Verify lidar is publishing, check clock sanitizer |
| Robot doesn't move | `ros2 topic echo /cmd_vel` | Check Nav2 lifecycle, controller plugin loaded |
| Planner fails | Check costmap in RViz | Wait for map, check global costmap has obstacle layer |
| Robot freezes at map edge | Controller logs | Fix collision check: don't treat NO_INFORMATION as obstacle |
| "Package not found" | `source install/setup.bash` | Source in every terminal after build |
| Build fails on C++ plugin | Check CMakeLists.txt | Ensure all Nav2 dependencies listed |
| Launch file can't find node | Check `setup.py` entry_points | Add missing executable to console_scripts |
