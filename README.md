# Voice-Activated Fetcher

A ROS 2 mobile-manipulator project for locating and fetching selected objects. The workspace contains packages for object perception, robot control, and sensor/input handling.

> **Current status:** Voice-related code is present, but the main `full_pipeline` launch currently takes the target object as typed keyboard input through `human_talk`. Voice-based target selection is not yet wired into that launch. The detector currently supports `bottle`, `cup`, and `sports ball`.
## Demo


https://github.com/user-attachments/assets/28f2929a-2715-42d3-9651-c93f55e05672


## Packages

- **`robo_vision`** — camera streaming and YOLO-based object detection. Detection publishes the target's horizontal offset and estimated distance for the mobile base.
- **`robo_control`** — mobile-base tracking, ultrasonic-to-arm command mapping, and a serial bridge for sending arm step commands.
- **`brain`** — target input, ultrasonic sensor publishing, and speech-related nodes.

## Requirements

- ROS 2 with a compatible `colcon` installation.
- A camera and the robot hardware used by the selected nodes (serial-connected mobile base, ultrasonic sensor, and/or arm controller).
- Python dependencies used by the nodes, including OpenCV, NumPy, Ultralytics, PySerial, and—if using the speech-related nodes—SpeechRecognition, gTTS, and playsound.
- The YOLO model file `yolov8n.pt` in the workspace root. The vision code loads this file using a relative path.

Install any missing ROS and Python dependencies in the environment used by ROS 2 before launching the nodes. Voice-related dependencies are only needed for the speech nodes.

## Build

From the workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run the pipeline

Run from the workspace root so the vision node can find `yolov8n.pt`:

```bash
ros2 launch robo_control full_pipeline.launch.py
```

When prompted, type a target object and press Enter. The launch starts the target-input node first, then starts the ultrasonic publisher, object detector, and mobile-base tracker after a target is published.

The launch defaults use hardware paths such as `/dev/usbcam` and `/dev/ttyUSB0`. Override them with launch arguments when your device paths differ. For example:

```bash
ros2 launch robo_control full_pipeline.launch.py \
  camera_device:=/dev/video0 \
  tracker_port:=/dev/ttyUSB0 \
  target_class:=bottle
```

Available target classes are `bottle`, `cup`, and `sports ball`.

The arm mapper (`ultrasonic_arm_mapper`) and arm serial bridge (`step_bridge_arm`) are included as separate nodes; the `full_pipeline.launch.py` launch file does not currently start them.

## ROS topics

The nodes communicate through ROS topics, including:

| Topic | Purpose |
| --- | --- |
| `/object_targets` | Selected object class |
| `/object_offset` | Target's horizontal image offset |
| `/object_distance` | Estimated target distance |
| `/ultrasonic_data` | Ultrasonic sensor readings |
| `/joint_steps_abs` | Absolute arm joint-step commands |

## Tests

```bash
colcon test
colcon test-result --verbose
```

The package test folders currently contain lint and style checks; they do not yet provide comprehensive hardware or end-to-end behavior tests.

## Safety

Test with the robot secured and the wheels/arm clear of people and obstacles. Verify serial ports and motion limits before enabling motors, and keep a way to stop the robot immediately.
