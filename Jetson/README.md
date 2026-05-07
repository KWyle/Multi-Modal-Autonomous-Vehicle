# Multi-Modal Autonomous Driving Alert System (ADAS)

A real-time, multi-threaded ADAS running on an **NVIDIA Jetson Orin Nano** and a **Raspberry Pi Pico 2 W**. The system fuses three sensing modalities — YOLOv8n object detection, Ultra Fast Lane Detection (UFLD), and HC-SR04 ultrasonic proximity — into a unified alert pipeline, with audio feedback delivered via a PWM buzzer on the Pico.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
  - [Components List](#components-list)
  - [Wiring & Pinout](#wiring--pinout)
- [Software](#software)
  - [Repository Structure](#repository-structure)
  - [Threading Model](#threading-model)
  - [Serial Protocol](#serial-protocol)
  - [Alert Types & Fusion Logic](#alert-types--fusion-logic)
  - [Buzzer Patterns](#buzzer-patterns)
  - [Dependencies](#dependencies)
- [Installation & Setup](#installation--setup)
  - [Jetson Orin Nano Setup](#jetson-orin-nano-setup)
  - [Raspberry Pi Pico 2 W Setup](#raspberry-pi-pico-2-w-setup)
- [Usage](#usage)
  - [Live Camera Mode](#live-camera-mode)
  - [Video File Mode](#video-file-mode)
  - [Video Recording](#video-recording)
- [Configuration Reference](#configuration-reference)
- [Known Limitations & Future Work](#known-limitations--future-work)

---

## Overview

This project implements a multi-modal ADAS alert system for a mobile autonomous platform, fusing three independent sensing modalities:

1. **Object Detection — YOLOv8n (Jetson GPU, TensorRT)**: A YOLOv8n model compiled as a TensorRT engine runs on the Jetson's GPU in a dedicated thread. Detected bounding boxes are fused with stereo depth data from the OAK-D Pro to estimate real-world distances to road-relevant objects.

2. **Lane Departure Detection — UFLD (Jetson GPU, TensorRT)**: An Ultra Fast Lane Detection model, also compiled as a TensorRT engine, runs in a second dedicated GPU thread. It identifies left and right lane boundaries and detects when the vehicle drifts toward either edge.

3. **Ultrasonic Proximity Sensing — HC-SR04 (Pico 2 W)**: The Pico continuously polls an HC-SR04 sensor and streams distance readings to the Jetson at ~10 Hz over USB serial. It also receives alert commands from the Jetson and drives a PWM buzzer with distinct audio patterns for each alert type.

All three streams are fused on the Jetson into a single prioritised `FusionResult` at 20 Hz, which is sent to the Pico as a serial command.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         NVIDIA Jetson Orin Nano                          │
│                                                                          │
│  ┌────────────────────┐                                                  │
│  │  Luxonis OAK-D Pro │  USB 3.0    ┌──────────────────────────────────┐ │
│  │  • RGB Camera      │────────────▶│         jetson_fusion.py         │ │
│  │  • Stereo Depth    │             │                                  │ │
│  └────────────────────┘             │  Main Thread                     │ │
│                                     │  • DepthAI 3.x pipeline          │ │
│  ┌──────────────────────────────┐   │  • Letterbox + frame dispatch    │ │
│  │  InferenceThread (GPU)       │◀──│  • Depth-bbox fusion             │ │
│  │  YOLOv8n TensorRT engine     │──▶│  • FusionEngine (20 Hz)          │ │
│  └──────────────────────────────┘   │  • OpenCV overlay + display      │ │
│                                     │  • Optional video recording      │ │
│  ┌──────────────────────────────┐   │                                  │ │
│  │  LaneInferenceThread (GPU)   │◀──│                                  │ │
│  │  UFLD TensorRT engine        │──▶│                                  │ │
│  └──────────────────────────────┘   │                                  │ │
│                                     │  SerialBridge Thread             │ │
│                                     │  • Read US,<inches> from Pico    │ │
│                                     │  • Write CMD,<type>,<param>      │ │
│                                     └──────────────┬───────────────────┘ │
└──────────────────────────────────────────────────────┼────────────────────┘
                                                       │ USB Serial
                                                       │ 115200 baud
                                                       │ /dev/ttyACM0
┌──────────────────────────────────────────────────────┼────────────────────┐
│                      Raspberry Pi Pico 2 W            │                    │
│                                                       │                    │
│  ┌────────────────────────────────────────────────────────────────────┐   │
│  │                           main.py                                  │   │
│  │  • Non-blocking serial RX  →  parse CMD,<type>,<param>             │   │
│  │  • Stream US,<inches>      ←  ultrasonic median of 5 (10 Hz)       │   │
│  │  • Buzzer state machine    →  PWM tone patterns                    │   │
│  └────────────────────────────────────────────────────────────────────┘   │
│          │ GP2 / GP3                                       │ GP15          │
│  ┌───────┴────────┐                              ┌─────────┴──────────┐   │
│  │  HC-SR04       │                              │  Passive PWM       │   │
│  │  Ultrasonic    │                              │  Buzzer / Speaker  │   │
│  └────────────────┘                              └────────────────────┘   │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware

### Components List

| Component | Details | Interface |
|---|---|---|
| Main Compute | NVIDIA Jetson Orin Nano | — |
| Microcontroller | Raspberry Pi Pico 2 W | USB Serial (115200 baud) |
| Depth Camera | Luxonis OAK-D Pro | USB 3.0 |
| Ultrasonic Sensor | HC-SR04 (or compatible) | GPIO — GP2 (Echo), GP3 (Trig) |
| Audio Feedback | Passive PWM Buzzer / Speaker | GPIO PWM — GP15 |

---

### Wiring & Pinout

#### Raspberry Pi Pico 2 W — HC-SR04 Ultrasonic Sensor

```
HC-SR04 Pin    →    Pico 2 W Pin
─────────────────────────────────
VCC            →    VBUS (5V)
GND            →    GND
TRIG           →    GP3
ECHO           →    GP2
```

> ⚠️ **Voltage Warning:** The HC-SR04 ECHO line outputs 5V logic. The Pico 2 W GPIO pins are **3.3V tolerant only**. Use a voltage divider (1 kΩ in series + 2 kΩ to GND on the ECHO line) or a dedicated logic level shifter to protect the Pico.

#### Raspberry Pi Pico 2 W — PWM Buzzer / Speaker

```
Buzzer Pin     →    Pico 2 W Pin
─────────────────────────────────
Positive (+)   →    GP15  (PWM-capable)
Negative (−)   →    GND
```

> 💡 The firmware uses a **passive buzzer** driven by PWM at varying frequencies (1200–3800 Hz). An active buzzer will not produce distinct tones. A small passive piezo buzzer or speaker with a current-limiting resistor (~100 Ω in series) is recommended.

#### Jetson Orin Nano — OAK-D Pro

```
OAK-D Pro      →    Jetson Orin Nano
─────────────────────────────────────
USB 3.0 cable  →    USB 3.0 port
```

> Use a USB 3.0 cable (blue connector) to ensure adequate bandwidth for simultaneous RGB + stereo depth streaming at 1920×1080.

#### Jetson Orin Nano — Raspberry Pi Pico 2 W

```
Pico 2 W       →    Jetson Orin Nano
─────────────────────────────────────
Micro-USB      →    USB port  (appears as /dev/ttyACM0)
```

The Pico enumerates as a USB CDC serial device. It typically appears as `/dev/ttyACM0`. If another USB serial device is present it may enumerate as `/dev/ttyACM1` — adjust `SERIAL_PORT` in `jetson_fusion.py` accordingly.

---

## Software

### Repository Structure

```
Multi-Modal-Autonomous-Vehicle/  (YOLO branch)
│
├── jetson_fusion.py        # Main entry point — run on the Jetson Orin Nano.
│                           # Manages three parallel threads (YOLO inference,
│                           # UFLD lane inference, serial bridge), runs the
│                           # DepthAI 3.x capture pipeline, fuses all sensor
│                           # data at 20 Hz, and renders a fullscreen OpenCV
│                           # overlay. Supports live camera and video file modes.
│
├── main.py                 # MicroPython firmware — deployed on the Pico 2 W.
│                           # Polls the HC-SR04 (median of 5 samples), streams
│                           # distance readings to the Jetson at ~10 Hz, and
│                           # drives the PWM buzzer with per-alert-type patterns.
│
├── models/
│   ├── yolov8n_640_nms.engine
│   │                       # YOLOv8n compiled as a TensorRT engine for the
│   │                       # Jetson GPU (640×640 input, includes NMS).
│   │                       # Inference runs on the Jetson GPU via Ultralytics.
│   │
│   └── ultra-fast-lane-det-culane.engine
│                           # Ultra Fast Lane Detection (UFLD) compiled as a
│                           # TensorRT engine. Detects up to 4 lane lines and
│                           # identifies departure from the two inner lanes.
│
└── car_simulation.py       # [UNUSED] Early-stage simulation/testing script.
                            # Not part of the active pipeline. Scheduled for
                            # removal in a future cleanup commit.
```

---

### Threading Model

`jetson_fusion.py` uses four concurrent execution contexts:

| Thread | Class | Role |
|---|---|---|
| Main thread | — | DepthAI frame capture, depth fusion, display, fusion decisions |
| YOLO thread | `InferenceThread` | YOLOv8n TensorRT inference on GPU; accepts letterboxed frames via `submit_frame()`, exposes results via `get_latest()` |
| Lane thread | `LaneInferenceThread` | UFLD TensorRT inference on GPU; preprocesses full frames to 800×288, runs postprocessing to extract lane polylines and departure status |
| Serial thread | `SerialBridge` | Background reader for `US,<inches>` lines from the Pico; exposes latest reading and sends `CMD` messages on state change |

If the UFLD engine file is not found or fails to load, a `DummyLaneInferenceThread` is substituted automatically — the system continues running with lane detection disabled rather than crashing.

---

### Serial Protocol

Communication between the Jetson and Pico uses USB CDC serial at **115200 baud**, newline-terminated ASCII messages.

| Direction | Format | Example | Description |
|---|---|---|---|
| Pico → Jetson | `US,<float_inches>\n` | `US,7.23\n` | Ultrasonic distance reading (~10 Hz) |
| Jetson → Pico | `CMD,<type>,<param>\n` | `CMD,PARK,72\n` | Alert command (sent on change only) |

**CMD types sent by the Jetson:**

| Command | Meaning |
|---|---|
| `CMD,NONE,0` | All clear — silence all alerts |
| `CMD,PARK,<0–100>` | Parking proximity; param = severity (100 = at minimum safe distance) |
| `CMD,COLLISION,1` | Camera detects slow-down threat (closest road object ≤ 2.0 m) |
| `CMD,COLLISION,2` | Full-stop threat (object ≤ 1.0 m, or camera slow-down escalated by ultrasonic) |
| `CMD,LANE,1` | Lane departure — left |
| `CMD,LANE,2` | Lane departure — right |

Commands are **deduplicated** on the Jetson side — a new serial message is only transmitted when the alert type or parameter changes.

---

### Alert Types & Fusion Logic

The `FusionEngine` runs at **20 Hz** and applies the following strict priority hierarchy:

```
Priority  Alert          Trigger Condition
────────  ─────────────  ──────────────────────────────────────────────────────
  1 (▲)   COLLISION,2   Camera: FULL_STOP  (closest road object ≤ 1.0 m)
                    OR   Camera: SLOW_DOWN  AND  ultrasonic ≤ 6 in
           COLLISION,1   Camera: SLOW_DOWN  (closest road object ≤ 2.0 m)
  2        PARK,<sev>    Camera: KEEP_SPEED  AND  ultrasonic ≤ 12 in
  3        LANE,<1|2>    No collision/park  AND  lane departure detected
  4 (▼)   NONE,0        All clear
```

**Fusion escalation** is the key cross-modal behaviour: a `COLLISION,1` (camera slow-down) is escalated to `COLLISION,2` (full stop) when the ultrasonic sensor simultaneously reads ≤ 6 inches, treating agreement between both sensors as a higher-confidence threat.

**Camera detections** are filtered to road-relevant COCO classes using a **perspective-aware trapezoidal corridor**: the horizontal acceptance window widens from 30–70% of frame width at the top of the road zone to 5–95% at the bottom, approximating a real-world forward driving corridor.

| Monitored COCO Classes | Vertical Filter |
|---|---|
| `person`, `bicycle`, `motorbike`, `car`, `bus`, `truck`, `train` | Bottom 45% of frame only (y > 0.55) |

**Lane departure** is detected by the UFLD model using the bottom 6 anchor rows of the two inner lane lines. Departure is flagged when the vehicle centre (0.5) drifts outside the lane boundary by more than `LANE_DEPART_MARGIN` (default: 0.08 normalised width).

---

### Buzzer Patterns

The Pico's non-blocking buzzer state machine produces distinct audio patterns for each alert type:

| Alert | Frequency | Pattern | Character |
|---|---|---|---|
| `PARK` (low severity) | 2000 Hz | 50 ms on / up to 550 ms off | Slow, relaxed parking beeps |
| `PARK` (high severity) | 2000 Hz | 50 ms on / down to 50 ms off | Rapid beeping as obstacle closes |
| `PARK` (severity = 100) | 2000 Hz | Continuous tone | At minimum safe distance |
| `COLLISION,1` | 3000 Hz | 120 ms on / 180 ms off | Steady urgent beeping |
| `COLLISION,2` | 3800 Hz | 220 ms on / 80 ms off | Fast, high-pitched alarm |
| `LANE` | 1200 Hz | Triple-pulse (80/80/80 ms) then 600 ms pause | Rhythmic directional tone |
| `NONE` | — | Silence | — |

---

### Dependencies

#### Jetson Orin Nano (Python 3.8+)

| Package | Purpose | Install |
|---|---|---|
| `depthai` | DepthAI 3.x SDK — OAK-D Pro pipeline and stereo depth | `pip install depthai` |
| `ultralytics` | YOLOv8n model wrapper for TensorRT engine loading | `pip install ultralytics` |
| `tensorrt` | TensorRT runtime for UFLD lane engine deserialisation | Installed with JetPack |
| `torch` (PyTorch) | GPU tensor management for UFLD inference buffers | Installed with JetPack |
| `opencv-python` | Frame processing, display overlay, and video I/O | `pip install opencv-python` |
| `numpy` | Depth ROI statistics and lane postprocessing | `pip install numpy` |
| `pyserial` | USB serial communication with Pico | `pip install pyserial` |

> 💡 `tensorrt` and `torch` are pre-installed with JetPack and should **not** be installed via pip on Jetson — use the JetPack-provided versions to ensure CUDA compatibility.

> 💡 Similarly, install OpenCV from the JetPack wheel or build from source with CUDA support rather than the PyPI binary for hardware-accelerated performance.

#### Raspberry Pi Pico 2 W (MicroPython)

All libraries are part of the **MicroPython standard library** — no additional packages need to be installed:

| Module | Purpose |
|---|---|
| `machine.Pin` | GPIO control for TRIG, ECHO, and BUZZER pins |
| `machine.PWM` | PWM output for buzzer tone generation |
| `machine.time_pulse_us` | Microsecond echo timing for the HC-SR04 |
| `uselect` | Non-blocking poll on stdin for serial RX |
| `sys.stdin` / `sys.stdout.buffer` | USB CDC serial communication |
| `utime` / `time` | Timing for ultrasonic sampling and buzzer state machine |

---

## Installation & Setup

### Jetson Orin Nano Setup

#### 1. Verify JetPack

Ensure the Jetson is running **JetPack 5.x or 6.x** (includes CUDA, TensorRT, and PyTorch). Check with:

```bash
cat /etc/nv_tegra_release
```

#### 2. Clone the Repository

```bash
git clone https://github.com/KWyle/Multi-Modal-Autonomous-Vehicle.git
cd Multi-Modal-Autonomous-Vehicle
git checkout YOLO
```

#### 3. (Recommended) Create a Virtual Environment

```bash
python3 -m venv venv --system-site-packages
source venv/bin/activate
```

> Use `--system-site-packages` so the venv can access JetPack-installed packages (`tensorrt`, `torch`, `cv2`).

#### 4. Install Python Dependencies

```bash
pip install depthai ultralytics numpy pyserial
# Do NOT pip install tensorrt, torch, or opencv — use JetPack versions
```

#### 5. Install OAK-D Pro udev Rules

Allows access to the OAK-D Pro without `sudo`:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

#### 6. Place TensorRT Model Engines

The system requires two pre-compiled TensorRT `.engine` files. By default, `jetson_fusion.py` expects them at:

```
/home/vehicles/Desktop/vehicles/models/yolov8n_640_nms.engine
/home/vehicles/Desktop/vehicles/models/ultra-fast-lane-det-culane.engine
```

Update `ENGINE_PATH` and `LANE_ENGINE_PATH` at the top of `jetson_fusion.py` if your paths differ.

> **Compiling engines:** TensorRT engines must be compiled on the target Jetson. Use the [Ultralytics export command](https://docs.ultralytics.com/modes/export/) for YOLOv8n and the [Luxonis blobconverter](https://blobconverter.luxonis.com/) or `trtexec` for UFLD.

#### 7. Verify Hardware

```bash
# Confirm OAK-D Pro is detected
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"

# Confirm Pico serial port
ls /dev/ttyACM*
```

---

### Raspberry Pi Pico 2 W Setup

#### 1. Flash MicroPython

Download the latest **MicroPython UF2 for Pico 2 W** from the [official MicroPython downloads page](https://micropython.org/download/RPI_PICO2_W/).

1. Hold **BOOTSEL** on the Pico and plug it into USB.
2. It mounts as `RPI-RP2`.
3. Drag and drop the `.uf2` file onto the drive. The Pico reboots into MicroPython.

#### 2. Deploy `main.py`

**Using Thonny IDE** (recommended):
1. Install [Thonny](https://thonny.org/) and set the interpreter to **MicroPython (Raspberry Pi Pico)**.
2. Open `main.py` from the repository root.
3. **File → Save as... → Raspberry Pi Pico** → save as `main.py`.

**Using `mpremote` (CLI):**
```bash
pip install mpremote
mpremote connect /dev/ttyACM0 cp main.py :main.py
```

> `main.py` runs automatically on every boot. On first power-up it prints `Pico ADAS firmware ready.` over USB serial.

#### 3. Wire the Hardware

Connect components to the Pico as described in the [Wiring & Pinout](#wiring--pinout) section.

---

## Usage

### Live Camera Mode

With `USE_VIDEO_FILE = False` (default), the system uses the OAK-D Pro as its live input. Ensure both the OAK-D Pro and Pico 2 W are connected to the Jetson, then run:

```bash
cd Multi-Modal-Autonomous-Vehicle
source venv/bin/activate
python3 jetson_fusion.py
```

The Pico starts automatically on power-up and immediately begins streaming ultrasonic data.

**Expected output:**

- A fullscreen OpenCV window (`ADAS Sensor Fusion`) showing the live RGB feed with bounding boxes (blue), depth-fused distance labels, lane line overlays (cyan = left, yellow = right), and a colour-coded alert banner in the bottom-left.
- The terminal prints a 2 Hz status summary, for example:

```
[14:32:07] road_dets=2 | ultrasonic=9.4in
  >>> Closest: person  dist=1.83m  conf=0.74
  - person       conf=0.74  dist=1.83m
  - car          conf=0.61  dist=4.10m
  LANE status=0 L=0.28 R=0.71
  CAMERA: SLOW_DOWN
  FUSION -> COLLISION, param=1
```

Press **`q`** in the display window or **`Ctrl+C`** in the terminal to stop. All threads and the camera pipeline shut down cleanly.

---

### Video File Mode

To run the full pipeline against a pre-recorded video file instead of the live camera:

1. Set `USE_VIDEO_FILE = True` in `jetson_fusion.py`.
2. Set `VIDEO_FILE_PATH` to the path of your `.mp4` file.
3. Run as normal. Depth data will not be available in video mode, but YOLO, lane detection, and ultrasonic fusion remain active.

---

### Video Recording

To record the annotated output video while running in live camera mode:

1. Set `RECORD_VIDEO = True` in `jetson_fusion.py`.
2. Set `RECORD_OUTPUT_PATH` to your desired output file path.
3. Set `RECORD_DURATION` to a maximum duration in seconds, or `0` for unlimited.

Recorded video is saved as an MP4 at `RECORD_FPS` (default: 20 fps) at 1920×1080 resolution.

---

## Configuration Reference

All tuneable constants are defined near the top of each file.

### `jetson_fusion.py`

| Constant | Default | Description |
|---|---|---|
| `ENGINE_PATH` | *(see file)* | Path to the YOLOv8n TensorRT `.engine` file |
| `LANE_ENGINE_PATH` | *(see file)* | Path to the UFLD TensorRT `.engine` file |
| `MODEL_INPUT_SIZE` | `640` | YOLOv8n input resolution (square) |
| `CONF_TH` | `0.5` | YOLO minimum detection confidence threshold |
| `IOU_TH` | `0.5` | YOLO NMS IoU threshold |
| `FRONT_STOP_DISTANCE` | `1.0` m | Camera distance triggering `COLLISION,2` |
| `FRONT_SLOW_DISTANCE` | `2.0` m | Camera distance triggering `COLLISION,1` |
| `ULTRA_MAX_INCHES` | `12.0` in | Ultrasonic range beyond which parking alerts are suppressed |
| `ULTRA_MIN_INCHES` | `3.0` in | Ultrasonic distance at which parking severity reaches 100 |
| `FUSION_ESCALATE_ULTRA_INCHES` | `6.0` in | Ultrasonic threshold for escalating `COLLISION,1` → `COLLISION,2` |
| `DECISION_HZ` | `20.0` | Fusion + serial command rate |
| `PRINT_HZ` | `2.0` | Console status print rate |
| `SERIAL_PORT` | `/dev/ttyACM0` | Pico USB serial device path |
| `SERIAL_BAUD` | `115200` | Serial baud rate |
| `LANE_CONF_TH` | `0.10` | Minimum UFLD per-anchor confidence (raised from 0.01 to suppress ghost detections) |
| `LANE_DEPART_MARGIN` | `0.08` | Normalised-width margin for lane departure detection |
| `DISPLAY_W / H` | `1920 / 1080` | Display and recording resolution |
| `SHOW_VIDEO` | `True` | Enable/disable the OpenCV overlay window |
| `USE_VIDEO_FILE` | `False` | `True` to run on a video file instead of live camera |
| `RECORD_VIDEO` | `False` | `True` to save the annotated output to an MP4 |
| `RECORD_DURATION` | `300` s | Max recording duration in seconds (`0` = unlimited) |

### `main.py` (Pico)

| Constant | Default | Description |
|---|---|---|
| `TRIG_PIN` | `3` | GPIO pin for ultrasonic TRIG |
| `ECHO_PIN` | `2` | GPIO pin for ultrasonic ECHO |
| `BUZZER_PIN` | `15` | GPIO pin for PWM buzzer output |
| `US_SAMPLE_COUNT` | `5` | Ultrasonic readings per cycle (median taken) |
| `US_REPORT_EVERY_MS` | `100` ms | Ultrasonic streaming interval (~10 Hz) |
| `LANE_ON_MS` | `80` ms | Lane alert pulse-on duration |
| `LANE_OFF_MS` | `80` ms | Lane alert pulse-off duration |
| `LANE_PAUSE_MS` | `600` ms | Pause between lane alert triple-pulse groups |

---

## Known Limitations & Future Work

- **TensorRT engine portability** — TensorRT `.engine` files are compiled for a specific GPU architecture and TensorRT version. Engines must be recompiled on the target Jetson if JetPack is updated or the model is changed.

- **`car_simulation.py`** — A legacy simulation/testing script not connected to the active pipeline. Scheduled for removal in a future cleanup commit.

- **Headless deployment** — `SHOW_VIDEO = True` requires a connected display or virtual framebuffer. Set `SHOW_VIDEO = False` in `jetson_fusion.py` for headless/embedded deployments.

- **Depth accuracy at range** — Stereo depth degrades beyond ~5–6 m. The `MAX_MM = 8000` cap reflects this; objects beyond 8 m will not trigger camera-based alerts.

- **Ultrasonic blind spot** — The HC-SR04 has a beam angle of ~15°. Objects outside the forward cone (e.g. to the sides) are not captured by the ultrasonic channel.

- **UFLD on curved roads** — The UFLD CULane-trained model performs best on straight and mildly curved roads. Performance may degrade on sharp bends or non-standard lane markings.

- **Camera orientation** — The frame rotation for upside-down mounting (`cv2.ROTATE_180`) is currently commented out. Uncomment the relevant line in `_run_live()` if the OAK-D Pro is mounted inverted.

---

## License

Copyright (c) 2025 Colorado State University

---

*NVIDIA Jetson Orin Nano · Raspberry Pi Pico 2 W · Luxonis OAK-D Pro · YOLOv8n (TensorRT) · UFLD (TensorRT) · MicroPython · DepthAI 3.x*
