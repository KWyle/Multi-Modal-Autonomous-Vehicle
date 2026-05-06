## 1) How the whole system works (end-to-end)

### Data flow (conceptual)
**Gazebo (simulation) or hardware drivers publish sensor data** ➜ perception nodes consume it ➜ decision node publishes a final command.

**Main pipeline in your repo:**
1. **Gazebo model** is defined by `car_description/models/car_chassis.sdf`
2. A **launch file** under `car_description/launch/` spawns the model and world
3. Perception package (`perception_inference`) runs:
   - `infer_node.py` (YOLO inference on camera frames)
   - `oak_detection_distance.py` (distance estimation using depth + detections)
   - `ultrasonic_monitor.py` (range ➜ safety stop boolean)
   - `fusion_decision_node.py` (final decision: STOP/SLOW/GO or similar)

---

## 2) Gazebo Vehicle Model: `car_description/models/car_chassis.sdf`

### What this file is responsible for
`car_chassis.sdf` is the **source of truth for the simulated vehicle body**. It defines:
- vehicle links/joints (chassis, wheels, steering)
- inertial / collision / friction properties
- sensor mounting transforms (camera/depth/ultrasonic)
- Gazebo sensor plugins (camera, depth, ray sensors)

### Gazebo Simulation
Your `car_description/launch/` files are responsible for:
- starting Gazebo with a world (often from `Worlds/`)
- spawning the SDF model into the world
- ensuring ROS to Gazebo bridges are running if needed (depending on Gazebo version)

### What to document inside the SDF
Add/confirm these sections (or document what already exists):
- **Frames / links**: `base_link` and sensor frames (camera link, ultrasonic link)
- **Sensor rates**: camera FPS, depth FPS, ultrasonic update rate
- **Sensor range/limits**:
  - depth near/far clip
  - ultrasonic min/max range + noise
- **Plugin outputs**: which ROS topics are produced

---

## 3) Perception Package: `src/perception_inference/`

This package contains all runtime logic for:
- ML inference
- distance estimation
- ultrasonic safety logic
- fusion/decision

### Key folders
- `perception_inference/models/`  
  Stores ML artifacts (YOLO, etc.)
- `perception_inference/launch/`  
  Launch files to run nodes together (in sim or hardware mode)
- `perception_inference/perception_inference/*.py`  
  The actual ROS2 Python nodes

---

## 4) Node-by-node behavior (how each piece works)

> The exact topic names/types depend on your code implementation.
> Use `ros2 topic list` + `ros2 topic info <topic>` to confirm.

### 4.1 `infer_node.py` — ML inference (YOLO)
**Purpose:** Runs object detection on incoming images and publishes detections.

**Typical behavior:**
1. Subscribes to an image topic (sim camera or OAK RGB)
2. Converts ROS image ➜ OpenCV/Numpy
3. Preprocesses frame to model input (e.g., resize/letterbox)
4. Runs YOLO inference (PyTorch/ONNX/etc.)
5. Postprocesses (confidence threshold + NMS)
6. Publishes detection results (boxes/classes/conf)

**What it produces for downstream nodes:**
- bounding boxes (pixel coordinates)
- class IDs / labels
- confidence scores
- timestamp synchronized with the input frame
---

### 4.2 `oak_detection_distance.py` — detection distance estimation (depth + boxes)
**Purpose:** Converts 2D detections into **approximate distance** using depth data.

**Typical behavior:**
1. Subscribes to:
   - detections output from `infer_node.py`
   - depth stream (OAK depth or simulated depth)
2. For each detection bounding box:
   - chooses a sampling method (center pixel, median in ROI, trimmed mean, etc.)
   - rejects invalid depth (0, NaN, out-of-range)
3. Publishes enriched detections including estimated distance (meters)

**Why this node matters:**
- It transforms “I see an object” into “I see an object **X meters away**,”
  which is essential for safety logic and planning decisions.
---

### 4.3 `ultrasonic_monitor.py` — ultrasonic safety monitor
**Purpose:** Converts raw ultrasonic ranges into a robust safety signal.

**Typical behavior:**
1. Subscribes to one or more ultrasonic topics
2. Filters noisy readings (debounce / moving average / median)
3. Compares distance to thresholds:
   - if any sensor < `stop_distance_m` ➜ publish STOP flag
4. Publishes a boolean safety topic
---

### 4.4 `fusion_decision_node.py` — final decision / command output
**Purpose:** Fuses perception + ultrasonic safety into a single output command.

**Typical behavior:**
1. Subscribes to:
   - ultrasonic stop boolean
   - distance detections from `oak_detection_distance.py`
2. Applies deterministic safety-first rules, e.g.:
   - If ultrasonic_stop = True ➜ `STOP`
   - Else if object within danger distance or in ROI ➜ `SLOW`
   - Else ➜ `GO`
3. Publishes a final command topic (string/custom msg)

**Recommended decision priority**
1. Ultrasonic hard-stop override
2. Vision-based proximity slowdown/stop
3. Clear path ➜ GO

**Common parameters:**
- `danger_distance_m`
- `slow_distance_m`
- `classes_of_interest` (person, vehicle, cone, etc.)
- ROI / corridor settings (optional)

---

## 5) Launching & Running
- Read ROS2 Launch Cheatsheet
