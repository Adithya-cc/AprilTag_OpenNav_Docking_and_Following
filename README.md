# Autodock Controller: Docking and Following Guide

ROS 2 package `apriltag_docking` (the folder is called `autodock_controller`). It gives the robot two abilities:

1. **Docking:** drive to a charging dock marked with an AprilTag and stop in front of it.
2. **Following:** follow a detected person at a set distance.

There are **two separate docking methods** in this package, and one following method. Pick one docking method. Do not run both at once, because both publish velocity commands.

| Ability | Method | Server side | Client side |
|---|---|---|---|
| Docking | **A. AprilTag controller** (own state machine) | `controller` node, service `autodock_controller/docking_service` | `docking_client`, or `ros2 service call` |
| Docking | **B. Nav2 docking server** | `docking_server` (`opennav_docking`), action `dock_robot` | `ros2 action send_goal`, plus `connection.py` feeding it the dock pose |
| Following | **Nav2 following server** | `following_server` (`opennav_following`), action `follow_object` | `ros2 action send_goal`, plus `person.py` feeding it the person pose |

---

## 1. What each file is for

### Launch files (`launch/`)

| File | What it starts | Use it for |
|---|---|---|
| `controller.launch.py` | `controller` node only, with `param/neuronbot.yaml` (accepts `params_file:=`) | **Docking method A.** The simplest launch. |
| `autodock_gazebo.launch.py` | `controller` node, plus RViz if `open_rviz:=true` | **Docking method A in simulation.** The AprilTag launch inside it is commented out, so start the tag node yourself. |
| `autodock_neuronbot.launch.py` | `controller` node, plus `apriltag_ros` `tag_realsense.launch.py` | **Docking method A on the real robot** with a RealSense camera. |
| `navigation_launch.py` | Nav2 servers **plus `docking_server` and `following_server`**, and a lifecycle manager | **Docking method B and Following.** This is the one that starts the two action servers. |
| `gazebo.launch.py` | Gazebo, robot (`robot_1`), Nav2 localization and navigation, `apriltag_node` | Simulation bring-up. It depends on a package called `eval_test` that is not in this zip, see section 8. |
| `navigtion_filter.launch.py` | Nav2 bringup, route server, keepout and speed-filter map servers | **Not needed for docking or following.** For keepout zones and route planning. |

### Nodes and scripts (`src/`)

| File | Role | Installed by `CMakeLists.txt`? |
|---|---|---|
| `controller.cpp`, `service_state.cpp`, `controller.h`, `common.h` | Docking method A: the `controller` node and its state machine and service | Yes (`controller`) |
| `service_client_node.cpp` | Client for the docking service | Yes (`docking_client`) |
| `connection.py` | **Dock pose bridge** for docking method B: turns AprilTag detections into `/detected_dock_pose` | Yes |
| `person.py` | **Person detector** for following: publishes `/detected_person` | Yes |
| `follow_client.py` | Python action client for following | **No**, and it has a bug, see section 8 |
| `object_follow.py` | Old PID follower | **No**, broken, do not use |
| `test_person.py` | Experimental copy of `person.py` (torso landmarks, publishes `/detecteds_person`) | No |
| `test_server.cpp` | ROS "hello world" publisher demo | No |

### Config and data
| File | Used by |
|---|---|
| `param/neuronbot.yaml` | `controller` (docking method A) |
| `param/nav2_params.yaml` | `navigation_launch.py`: Nav2, **`docking_server`** and **`following_server`** settings |
| `param/nav2_filter.yaml` | `navigtion_filter.launch.py` only |
| `param/deeplabv3.tflite` | `person.py` (person segmentation) |
| `param/yolo11n.pt` | Not used by any file here |
| `maps/map_warehouse.yaml/.pgm` | `gazebo.launch.py` |
| `rviz/apriltag.rviz` | `autodock_gazebo.launch.py` |

---

## 2. Prerequisites

| Needed | Why |
|---|---|
| `autodock_msgs` package (**not in this zip**) | Defines `srv/Docking` and `msg/CurrentState`. The package cannot build without it. |
| `apriltag_ros` and `apriltag_msgs` | Detect the tag and publish the TF `camera → dock_frame` and `/detections`. |
| Nav2 with **`opennav_docking`** and **`opennav_following`** | The docking and following servers. These come with newer Nav2 releases (the docking server arrived after Humble), so check `ros2 pkg list \| grep opennav`. If they are missing, install a newer ROS distro or build Nav2 from source. |
| `nav2_msgs`, `opennav_following_msgs` | Action types for the CLI and clients. |
| `mediapipe` (pip), OpenCV, `cv_bridge`, `numpy<2` | `person.py` |
| A camera publishing an image, camera info and (for following) a point cloud | See section 5. |
| TF tree `map → odom → base_link` | All methods need `odom → base_link`. Method B and following also use `map`. |

Build:
```bash
cd ~/autodock_ros2_ws
colcon build --packages-select autodock_msgs apriltag_docking
source install/setup.bash
```

---

## 3. Docking method A: AprilTag controller (service)

### How it works
The `controller` node reads TF: `odom → base_link` and `dock_frame → base_link`. `dock_frame` is the tag frame that `apriltag_ros` publishes. It runs a state machine at 30 Hz and publishes velocity on `cmd_vel`.

![](apriltag_docking/readme_resource/apriltag_docking.gif)

| State | What the robot does |
|---|---|
| (empty) | Idle. Nothing is published. |
| `searching` | Turns by `default_turn` until it sees the tag for `lost_tag_max` camera callbacks. |
| `blind` | The tag is seen at a very sharp angle. It turns away and moves sideways, then returns to `searching`. |
| `centering` | Turns until it faces the tag within an angle limit. If this takes more than `max_center_count` tries, it returns to `searching`. |
| `approach` | Moves forward in steps of `jog_distance`, re-centering if the angle drifts. |
| `final_approach` | Drives the last distance down to `finish_distance` and aligns the heading with the dock. |
| `docked` | Stops. Sending `start` again resets the state. |

### The service (server side)
- **Name:** `/autodock_controller/docking_service`
- **Type:** `autodock_msgs/srv/Docking`
- **Request:** `service` (string): `"start"` or `"cancel"`
- **Response:** `service_success` (bool)

| Request | Accepted when | Effect |
|---|---|---|
| `start` | The controller is idle (a `docked` state is reset first) | Goes to `searching`, returns `true` |
| `cancel` | Any time | Stops the robot, goes to idle, returns `true` |
| anything else | | Returns `false` |

### Published topics
| Topic | Type | Content |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | Velocity commands while a docking is active |
| `autodock_controller/current_state` | `autodock_msgs/msg/CurrentState` | `docking_state` and `action_state` |

### Client side
The package ships a client, `docking_client`. It waits for the service, sends one request and exits. The request string comes from the ROS parameter `docking` (default `start`).

### Step by step
1. **Start the robot or simulation** so that `/odom`, TF and the camera image are available.
2. **Start the AprilTag detector** so the tag frame `dock_frame` appears in TF. (The `tag_frame` parameter of the controller and the frame name set in `apriltag_ros` must match.)
   - Real robot: `ros2 launch apriltag_docking autodock_neuronbot.launch.py` starts both the controller and the tag detector. Then skip step 3.
   - Simulation: start `apriltag_ros` yourself (its `tag_gazebo.launch.py` is commented out in `autodock_gazebo.launch.py`).
3. **Start the controller** (the docking server):
   ```bash
   ros2 launch apriltag_docking controller.launch.py
   # or, with RViz in simulation
   ros2 launch apriltag_docking autodock_gazebo.launch.py open_rviz:=true
   ```
4. **Check that the service exists:**
   ```bash
   ros2 service list | grep docking_service
   ros2 service type /autodock_controller/docking_service
   ```
5. **Call the service (client).** Either use the shipped client:
   ```bash
   ros2 run apriltag_docking docking_client                         # start docking
   ros2 run apriltag_docking docking_client --ros-args -p docking:=cancel
   ```
   or call it from the command line:
   ```bash
   ros2 service call /autodock_controller/docking_service autodock_msgs/srv/Docking "{service: 'start'}"
   ros2 service call /autodock_controller/docking_service autodock_msgs/srv/Docking "{service: 'cancel'}"
   ```
   The client prints `Send service request(docking:=start) successfully.` or `... failed.`
6. **Watch the progress:**
   ```bash
   ros2 topic echo /autodock_controller/current_state
   ```
   The robot is docked when `docking_state` becomes `docked`, and the controller logs `Finish Docking!`.

### Tuning (`param/neuronbot.yaml`)
| Parameter | Default in file | Meaning |
|---|---|---|
| `cmd_vel_angular_rate`, `cmd_vel_linear_rate` | 0.20, 0.20 | Turn and drive speeds |
| `default_turn` | 1.0 rad | Turn step while searching |
| `final_approach_distance` | 1.0 m | Distance where the robot slows to half speed |
| `finish_distance` | 0.4 m | Distance from the dock when docking is finished |
| `jog_distance` | 0.2 m | Forward step in `approach` |
| `lost_tag_max` | 5 | Tag-lost tolerance in search and approach |
| `max_center_count` | 10 | Tag-lost tolerance in centering |
| `tune_angle`, `blind_angle` | 0.42, 0.8 rad | Approach and blind-zone angles |
| `tag_frame` | `dock_frame` | TF frame of the tag |

---

## 4. Docking method B: Nav2 docking server (action)

### How it works
`navigation_launch.py` starts `docking_server` (`opennav_docking`) with the `docking_server:` section of `param/nav2_params.yaml`:
- `dock_plugins: ['nova_carter_dock']` with `SimpleChargingDock`
- `use_external_detection_pose: true`: the final alignment uses a detected dock pose
- `docks: ['home_dock']` at `map` pose `[2.0, -2.0, 1.5708]`
- `base_frame: base_link`, `fixed_frame: odom`, `staging_x_offset: -0.5`, `max_retries: 3`

The server navigates to a staging pose near the dock, then drives in using the **detected dock pose**. `connection.py` supplies that pose: it subscribes to `/detections` (AprilTag), looks up the TF `camera_color_optical_frame → dock_frame` (parameters `camera_frame`, `tag_frame`) and publishes it as `/detected_dock_pose` (`geometry_msgs/PoseStamped`).

### Step by step
1. Start the robot or simulation, localization (AMCL and map) and the camera.
2. Start the AprilTag detector (`apriltag_ros`), so `/detections` and the `dock_frame` TF exist.
3. Start Nav2 with the docking and following servers. **Pass the params file explicitly**, because the default is Nav2's own file, which has no dock settings:
   ```bash
   ros2 launch apriltag_docking navigation_launch.py use_sim_time:=true \
     params_file:=$(ros2 pkg prefix apriltag_docking)/share/apriltag_docking/param/nav2_params.yaml
   ```
   The lifecycle manager activates `docking_server` and `following_server` along with the other servers.
4. Start the dock pose bridge:
   ```bash
   ros2 run apriltag_docking connection.py
   # if your camera frame name differs
   ros2 run apriltag_docking connection.py --ros-args -p camera_frame:=<your_optical_frame>
   ```
5. Check that the action servers are up:
   ```bash
   ros2 action list          # /dock_robot and /undock_robot should be listed
   ros2 topic echo /detected_dock_pose --once
   ```
6. **Send the dock goal (client).** This package has no Python or C++ client for it, so use the CLI:
   ```bash
   ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
     "{use_dock_id: true, dock_id: 'home_dock', navigate_to_staging_pose: true}" --feedback
   ```
   Undock:
   ```bash
   ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot "{dock_type: 'nova_carter_dock'}"
   ```

Use `navigate_to_staging_pose: false` if the robot is already near the dock.

---

## 5. Following a person

### How it works
- `person.py` (node `person_follower`) subscribes to the camera image `/camera_depth_frame/image_raw` and the point cloud `/camera_depth_frame/points`. It finds the person with MediaPipe pose and segmentation (`deeplabv3.tflite`), reads the 3D position of the person from the point cloud, and publishes:
  - `/detected_person` (`PoseStamped`, in the camera frame), which the following server tracks
  - the TF `detected_person` (child of the camera frame)
  - `/person_marker` (RViz sphere)
  - `/dummy/cmd_vel` (a dummy velocity output, not used by Nav2)
- `following_server` (`opennav_following`) subscribes to the pose topic given in the goal and drives the robot to keep `desired_distance: 1.0` m from it. Main settings in `nav2_params.yaml`:

| `following_server` parameter | Value | Meaning |
|---|---|---|
| `desired_distance` | 1.0 m | Keep this distance from the person |
| `detection_timeout` | 6.0 s | Give up after this long without a detection |
| `search_by_rotating` | true | Rotate to look for the person after a loss |
| `max_retries` | 6 | Retries before the goal fails |
| `static_object_timeout` | -1.0 | Follow indefinitely |
| `skip_orientation` | true | Only follow the position, ignore the person's heading |
| `base_frame`, `fixed_frame` | `base_link`, `odom` | Frames used for control |

### Step by step
1. Start the robot or simulation with the depth camera, and a TF tree that reaches the camera frame.
2. Start Nav2 with the following server (same command as docking method B, step 3). Docking servers start with it, which is fine.
3. Install the Python packages and fix the model path (see section 8):
   ```bash
   pip3 install mediapipe "numpy<2"
   ```
4. Start the person detector:
   ```bash
   ros2 run apriltag_docking person.py
   ros2 topic echo /detected_person --once      # should show a pose when a person is in view
   ```
5. **Send the follow goal (client)**:
   ```bash
   ros2 action send_goal /follow_object opennav_following_msgs/action/FollowObject \
     "{pose_topic: '/detected_person', tracked_frame: '', max_duration: {sec: -1, nanosec: 0}}" --feedback
   ```
   `pose_topic` is used when you follow a topic, so `tracked_frame` stays empty. `max_duration` of -1 means follow without a time limit. The feedback shows the state (`INITIAL_PERCEPTION`, `CONTROLLING`, `STOPPING`, `RETRY`), the following time and the retry count.
6. **Stop following:** press `Ctrl+C` in the `send_goal` terminal to cancel the goal.

`follow_client.py` is the Python version of step 5 (same goal). It is not installed and does not work as it is, see section 8.

---

## 6. Quick reference

### Services, actions and key topics
| Interface | Name | Type | Direction |
|---|---|---|---|
| Service | `/autodock_controller/docking_service` | `autodock_msgs/srv/Docking` | server: `controller`, client: `docking_client` |
| Action | `/dock_robot` | `nav2_msgs/action/DockRobot` | server: `docking_server` |
| Action | `/undock_robot` | `nav2_msgs/action/UndockRobot` | server: `docking_server` |
| Action | `/follow_object` | `opennav_following_msgs/action/FollowObject` | server: `following_server` |
| Topic | `/detections` | `apriltag_msgs/AprilTagDetectionArray` | published by `apriltag_ros`, read by `connection.py` |
| Topic | `/detected_dock_pose` | `geometry_msgs/PoseStamped` | `connection.py` → `docking_server` |
| Topic | `/detected_person` | `geometry_msgs/PoseStamped` | `person.py` → `following_server` |
| Topic | `autodock_controller/current_state` | `autodock_msgs/CurrentState` | `controller` |
| Topic | `cmd_vel` | `geometry_msgs/Twist` | `controller` (method A); Nav2 pipeline for B and following |

### What to start, in order
| Goal | 1 | 2 | 3 | 4 | Then call |
|---|---|---|---|---|---|
| **Dock (method A)** | Robot / Gazebo + camera | AprilTag node | `controller.launch.py` | | `docking_client` or `ros2 service call` |
| **Dock (method B)** | Robot / Gazebo + camera + localization | AprilTag node | `navigation_launch.py` (with `params_file`) | `connection.py` | `ros2 action send_goal /dock_robot ...` |
| **Follow person** | Robot / Gazebo + depth camera | `navigation_launch.py` (with `params_file`) | `person.py` | | `ros2 action send_goal /follow_object ...` |

### Useful checks
```bash
ros2 node list
ros2 service list | grep docking
ros2 action list
ros2 topic hz /detected_person
ros2 run tf2_ros tf2_echo base_link dock_frame        # the tag must be visible
ros2 lifecycle get /docking_server                    # should be "active"
```

---

## 7. Troubleshooting
| Symptom | Likely cause and fix |
|---|---|
| `docking_client` prints `service not available, waiting again...` | The `controller` node is not running. Start `controller.launch.py`. |
| Service returns `failed` | The controller is not idle (already docking), or the request text is not `start` or `cancel`. Send `cancel`, then `start`. |
| Robot just spins and never finds the tag | `dock_frame` is not in TF. Check `apriltag_ros` and that its frame name equals `tag_frame` in `neuronbot.yaml`. |
| `/dock_robot` is missing | `navigation_launch.py` was not started, or your Nav2 does not include `opennav_docking`. |
| Docking server refuses the goal, "no dock pose" | `connection.py` is not running, or `camera_frame` does not match your camera's optical frame. |
| Following server never leaves `INITIAL_PERCEPTION` | `/detected_person` is not publishing. Check `person.py`, its camera topics and the model path. |
| Nav2 nodes stay inactive | Check the lifecycle manager log. All names in its `node_names` must exist. |
| Nodes behave strangely in simulation | Launch with `use_sim_time:=true` everywhere. |

---

## 8. Known issues in the files (not changed)
1. **`autodock_msgs` is missing from the zip.** Both `controller` and `docking_client` need it, so build it first.
2. **`gazebo.launch.py` needs the package `eval_test`** (world, `turtlebot_burger.urdf.xacro`, `localization_launch.py`, `navigation_launch.py`). It starts **`eval_test`'s** navigation launch, not this package's, so it does **not** start `docking_server` or `following_server`. Start `navigation_launch.py` of this package separately, as in section 4.
3. In `gazebo.launch.py` the `apriltag_node` remapping builds the topic name from `LaunchConfiguration` objects inside an f-string, so the camera topic does not resolve. Start the tag detector separately with the correct topics.
4. **`person.py` loads `deeplabv3.tflite` from a hard-coded path** (`/home/adithya-si3510/autodock_ros2_ws/src/apriltag_docking/autodock_controller/param/deeplabv3.tflite`). Change it to the file's location on your machine.
5. **`follow_client.py` does nothing as written**: `main()` only spins and never calls `send_goal()`, and `send_goal()` calls itself at its end. It is also not listed in `CMakeLists.txt`. Use the `ros2 action send_goal` command until it is fixed.
6. `object_follow.py` overwrites the incoming message with an empty one and reads `msg.data` from a `PoseStamped`, so it cannot work. It is not installed.
7. `navigtion_filter.launch.py` lists a node named `'g'` in its lifecycle manager, which does not exist, so that manager cannot finish starting. Not needed for docking or following.
8. `param/nav2_params.yaml` has `home_dock` at the fixed pose `[2.0, -2.0, 1.5708]` in the `map` frame. Change it to where your dock really is.
9. `package.xml` names the package `apriltag_docking`, while the folder and the launch comments call it `autodock_controller`. Always use `apriltag_docking` in commands.
