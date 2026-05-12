# 🚗 CARLA Autonomous Driving Perception Pipeline

A lightweight autonomous driving perception stack built on the [CARLA Simulator](https://carla.org/), featuring real-time object detection, lane detection, and ultrasonic-style obstacle sensing — all powered by Python.

---

## 📦 Tech Stack

| Component | Technology |
|---|---|
| Simulator | CARLA 0.9.16 |
| Object Detection | YOLOv8n (Ultralytics) |
| Lane Detection | OpenCV |
| Obstacle Sensing | CARLA Obstacle Sensors (front-left & front-right) |
| Language | Python 3.12 |

---

## 📁 Project Directory

```
C:..\Senior Design\Carla\CARLA_Latest
```

```
CARLA_Latest/
├── .venv/
├── PythonAPI/
├── camera_lane_yolo_ultrasonic_test.py
├── test_connection.py
├── autopilot_view_test.py
└── README.md
```

---

## ⚙️ Environment Setup

### 1. Create Virtual Environment

```powershell
py -3.12 -m venv .venv
```

### 2. Activate Virtual Environment

```powershell
.\.venv\Scripts\Activate.ps1
```

> **Note:** If PowerShell blocks execution, run the following first, then activate again:
> ```powershell
> Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
> .\.venv\Scripts\Activate.ps1
> ```

### 3. Install Dependencies

Upgrade pip tools:
```powershell
python -m pip install --upgrade pip setuptools wheel
```

Install the CARLA Python API:
```powershell
python -m pip install ".\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"
```

Install additional packages:
```powershell
python -m pip install ultralytics opencv-python numpy pygame
```

---

## 🚀 Running the Simulation

Open two separate terminals.

**Terminal 1 — Launch CARLA:**
```powershell
cd "..\Senior Design\Carla\CARLA_Latest"
.\CarlaUE4.exe -ResX=1920 -ResY=1080
```

**Terminal 2 — Run the Perception Stack:**
```powershell
cd "..\Senior Design\Carla\CARLA_Latest"
.\.venv\Scripts\Activate.ps1
python .\camera_lane_yolo_ultrasonic_test.py
```

---

## 🕹️ Controls

| Key | Action |
|---|---|
| `Q` | Quit the OpenCV window |

---

## ✨ Features

### Vehicle
- **Ego vehicle:** Tesla Model 3
- CARLA autopilot enabled
- Traffic Manager integration

### RGB Front Camera

Used for YOLOv8n object detection, lane detection, and OpenCV visualization.

```python
camera_bp.set_attribute("image_size_x", "1280")
camera_bp.set_attribute("image_size_y", "720")
```

### Ultrasonic-Style Obstacle Sensors

Two obstacle sensors simulate short-range proximity detection:
- **Front-left** — lateral obstacle awareness
- **Front-right** — lateral obstacle awareness

### Object Detection (YOLOv8n)

```
CARLA Camera → NumPy/OpenCV Frame → YOLOv8n → Bounding Boxes
```

### Lane Detection

Lane detection pipeline using OpenCV:
1. Grayscale conversion
2. Gaussian blur
3. Canny edge detection
4. Region-of-interest masking
5. Hough line transform

**Outputs:** left lane, right lane, lane center offset

### Traffic System

Traffic vehicles are spawned automatically via CARLA's Traffic Manager:

```python
spawn_traffic(world, bp_lib, tm, actors, num_vehicles=60)
```

Pedestrians can also be added:

```python
spawn_pedestrians(world, bp_lib, actors, num_walkers=40)
```

---

## 📷 Spectator Camera Modes

### Third-Person Follow Camera

```python
vehicle_tf = vehicle.get_transform()
vehicle_loc = vehicle_tf.location
forward = vehicle_tf.get_forward_vector()

spectator_location = carla.Location(
    x=vehicle_loc.x - forward.x * 10.0,
    y=vehicle_loc.y - forward.y * 10.0,
    z=vehicle_loc.z + 4.0
)
spectator_rotation = carla.Rotation(
    pitch=-15.0,
    yaw=vehicle_tf.rotation.yaw,
    roll=0.0
)
spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))
```

### Front-of-Car Camera View

```python
vehicle_tf = vehicle.get_transform()
vehicle_loc = vehicle_tf.location
forward = vehicle_tf.get_forward_vector()

spectator_location = carla.Location(
    x=vehicle_loc.x + forward.x * 2.5,
    y=vehicle_loc.y + forward.y * 2.5,
    z=vehicle_loc.z + 1.2
)
spectator_rotation = carla.Rotation(
    pitch=-5.0,
    yaw=vehicle_tf.rotation.yaw,
    roll=0.0
)
spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))
```

---

## ⚡ Performance Tips

If you're experiencing low FPS, try the following optimizations:

**Reduce camera resolution:**
```python
camera_bp.set_attribute("image_size_x", "640")
camera_bp.set_attribute("image_size_y", "480")
```

**Run YOLO less frequently:**
```python
if frame_count % 5 == 0:   # instead of % 3
```

**Lower YOLO inference size:**
```python
results = model.predict(
    source=frame,
    imgsz=480,
    conf=0.35,
    verbose=False
)
```

---

## 🔄 Full Pipeline

```
CARLA Simulator
    ↓
RGB Camera Sensor
    ↓
NumPy / OpenCV Frame
    ↓
YOLOv8n Object Detection
    ↓
Lane Detection
    ↓
Ultrasonic Sensor Readings
    ↓
Merged Visualization
```

---

## 🔭 Future Improvements

- [ ] Steering control using lane center offset
- [ ] Automatic braking using YOLO detections
- [ ] Pedestrian avoidance
- [ ] Side collision avoidance
- [ ] Multi-camera support
- [ ] Dataset recording
- [ ] ROS2 integration
- [ ] Jetson deployment
- [ ] Semantic segmentation
- [ ] Depth estimation
- [ ] Sensor fusion logic
