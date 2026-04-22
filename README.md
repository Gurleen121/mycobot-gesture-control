# MyCobot 280 Pi — Gesture-Controlled Pick-and-Place Simulation

A gesture-controlled pick-and-place pipeline for the **Elephant Robotics MyCobot 280 Pi**
simulated in **PyBullet**. Hold up 1, 2, or 3 fingers in front of your laptop webcam
and the robot picks the corresponding box, navigates around a static obstacle, drops it
at a fixed target zone, and returns home.

Box identity is determined live by a phone camera running **ArUco marker detection** —
shuffle the boxes and the robot still picks the right one.

This simulation was developed alongside a hardware implementation on the real
MyCobot 280 Pi. The simulation uses ArUco markers for box identification in a
controlled environment; the hardware implementation was validated separately on
the physical arm.

---

## Demo

https://github.com/user-attachments/assets/7a845162-f15a-4807-9872-1b1a8109728d

---

## How It Works

### 1. ArUco Scan (startup gate)
The phone camera continuously scans for ArUco markers (IDs 1, 2, 3 from
`DICT_4X4_50`) pasted on the three physical boxes. Detected markers are sorted
by their pixel X-centre (leftmost → slot 0, middle → slot 1, rightmost →
slot 2). The internal `blocks` mapping is remapped so each marker ID points to
the correct PyBullet body regardless of physical arrangement. **The robot will
not respond to any gesture until all three markers are detected.**

### 2. Gesture Detection
MediaPipe Hands tracks the right hand on the laptop webcam. Index, middle, and
ring fingertips (landmarks 8, 12, 16) are compared against their PIP joints to
count 1–3 raised fingers. The gesture must be held stable for **1 second**
before triggering.

### 3. Pick-and-Place Sequence
Joint angles are linearly interpolated over 600 steps (~1.2 s). Sequence:

```
open gripper → above box → grip pose → attach constraint
→ avoid obstacle waypoint → drop zone → release → home
```

Gripping is implemented via a PyBullet fixed constraint between the end-effector
link and the target body. After the sequence, stale camera frames are flushed
so the gesture feed is live immediately on return.

### 4. Slot Remapping
After each successful scan, PyBullet 3D debug text above each box updates to
show the detected ArUco ID — the simulation reflects what the camera actually saw.

---

## System Architecture

```
┌─────────────────────────────────────────────────┐
│                    LAPTOP                       │
│                                                 │
│  ┌─────────────────┐   ┌─────────────────────┐  │
│  │ Webcam          │   │ Phone IP Camera     │  │
│  │ MediaPipe Hands │   │ cv2.aruco           │  │
│  │ finger count    │   │ DICT_4X4_50         │  │
│  │ 1 / 2 / 3       │   │ sort by X-centre    │  │
│  └────────┬────────┘   └──────────┬──────────┘  │
│           │ gesture               │ slot map     │
│           └──────────┬────────────┘             │
│                      ▼                          │
│            ┌─────────────────┐                  │
│            │  run_simulation │                  │
│            │  1s debounce    │                  │
│            │  blocks remap   │                  │
│            └────────┬────────┘                  │
│                     ▼                           │
│            ┌─────────────────┐                  │
│            │  PyBullet GUI   │                  │
│            │  URDF + STL     │                  │
│            │  joint interp   │                  │
│            │  constraint grip│                  │
│            └─────────────────┘                  │
└─────────────────────────────────────────────────┘
```

---

## CV Techniques Used

| Technique | Purpose |
|---|---|
| **ArUco marker detection** (`cv2.aruco`) | Identifies which physical box occupies which slot; enables slot remapping when boxes are shuffled |
| **MediaPipe Hands** | 21-landmark skeleton-based finger counting; right-hand only; 1-second stability gate |

---

## File Structure

```
mycobot-gesture-control/
├── run_simulation.py        # Main entry point
├── requirements.txt         # Python dependencies
├── README.md
├── LICENSE
├── urdf/
│   └── mycobot_280_pi.urdf  # Full 6-DOF robot description
└── meshes/
    ├── base_link.stl
    ├── Base_1.stl
    ├── Shoulder_1.stl
    ├── Elbow_1.stl
    ├── Elbow_2.stl
    ├── Wrist_1.stl
    ├── End_Effector_1.stl
    ├── gripper_2.0_1.stl
    ├── gripper_2.0_2.stl
    └── rev_color_senser_v3_1.stl
```

---

## Setup

### Prerequisites
- Python 3.8+
- Laptop webcam
- Android phone running **IP Webcam** app, on the same WiFi as the laptop
- 3 boxes with ArUco markers (IDs 1, 2, 3) printed from [chev.me/arucogen](https://chev.me/arucogen/)
  — select Dictionary: 4x4, print at ~80mm, paste with white border visible

### Install dependencies

```bash
pip install opencv-contrib-python mediapipe pybullet numpy
```

> Use `opencv-contrib-python` — **not** `opencv-python`. The ArUco module
> (`cv2.aruco`) is only in the contrib build.

### Configure phone IP

Open `run_simulation.py` and update line 189:

```python
PHONE_IP = "http://<your-phone-ip>:8080/video"
```

---

## Running

```bash
cd mycobot-gesture-control
python3 run_simulation.py
```

### Startup sequence
1. PyBullet GUI opens — robot at home position, 3 coloured boxes and cylinder obstacle visible.
2. Two OpenCV windows appear: laptop webcam (gesture) and phone camera (ArUco scan).
3. Point phone at all three boxes until overlay shows `ArUco detected: 3/3` and terminal prints the slot mapping.
4. Show 1, 2, or 3 fingers on the laptop webcam and hold for 1 second.

### Controls

| Gesture (right hand) | Action |
|---|---|
| 1 finger (hold 1s) | Pick box with ArUco ID 1 |
| 2 fingers (hold 1s) | Pick box with ArUco ID 2 |
| 3 fingers (hold 1s) | Pick box with ArUco ID 3 |
| ESC | Quit |

> Boxes can be placed in any left-to-right order — the ArUco scan remaps
> the simulation automatically each time.

---

## Dependencies

| Package | Purpose |
|---|---|
| `opencv-contrib-python` | ArUco detection, camera capture, frame processing |
| `mediapipe` | Hand landmark detection and finger counting |
| `pybullet` | Physics simulation, URDF loading, joint control |
| `numpy` | Marker corner averaging, interpolation |

---

## License

MIT — see [LICENSE](LICENSE).
