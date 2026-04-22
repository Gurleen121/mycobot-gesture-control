#!/usr/bin/env python3
"""
MyCobot 280 Pi - ArUco Marker System
Camera 1 (Laptop): Detects your hand gesture (1, 2, 3 fingers)
Camera 2 (Phone):  Reads ArUco markers on boxes (ID 1, 2, 3)
                   and maps them to PyBullet positions by left-to-right order.

CV Role: Phone camera detects which physical box (by ArUco ID) is sitting
         at which position (left/middle/right). This remaps the simulation
         so the robot always picks the correct box regardless of placement.
"""
import cv2
import cv2.aruco as aruco
import mediapipe as mp
import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import os

# === PATHS ===
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(PROJECT_DIR, "urdf", "mycobot_280_pi.urdf")
MESH_PATH = os.path.join(PROJECT_DIR, "meshes")

print("=" * 70)
print("  🤖 ARUCO MARKER SYSTEM")
print("=" * 70)
print("  Camera 1 (Laptop): Your hand gestures (1/2/3 fingers)")
print("  Camera 2 (Phone):  ArUco marker detection (IDs 1, 2, 3)")
print("  CV maps physical box positions → simulation picks correctly")
print("=" * 70)

if not os.path.exists(URDF_PATH):
    print("❌ URDF not found at:", URDF_PATH)
    exit(1)

# === PyBullet Setup ===
physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetSimulation()

pybullet_data_path = pybullet_data.getDataPath()
p.setAdditionalSearchPath(pybullet_data_path)
p.setAdditionalSearchPath(PROJECT_DIR)
p.setAdditionalSearchPath(MESH_PATH)

p.loadURDF(os.path.join(pybullet_data_path, "plane.urdf"))
p.setGravity(0, 0, -9.8)
p.setPhysicsEngineParameter(numSolverIterations=200)
p.setTimeStep(1 / 240)

p.resetDebugVisualizerCamera(
    cameraDistance=1.5, cameraYaw=0, cameraPitch=-20,
    cameraTargetPosition=[0.0, 0.3, 0.2]
)

# === Load Robot ===
robot = p.loadURDF(URDF_PATH, [0, 0, 0],
                   p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
p.resetJointState(robot, 3, math.radians(90))
p.resetJointState(robot, 4, math.radians(90))
print("✅ Robot loaded")

# === Joint Setup ===
def get_movable_joints():
    joints = []
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            joints.append(i)
    return joints

joints = get_movable_joints()
ee = p.getNumJoints(robot) - 1
home_joints = [0.0, 0.0, math.radians(90), math.radians(90), 0.0, 0.0]

# === Movement Functions ===
def move_joints(angles, steps=600):
    start = [p.getJointState(robot, j)[0] for j in joints]
    angles_padded = angles[:len(joints)] + [0] * (len(joints) - len(angles))
    for i in range(steps):
        alpha = i / steps
        target = [start[k] + alpha * (angles_padded[k] - start[k])
                  for k in range(len(joints))]
        for j, t in zip(joints, target):
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, t, force=500)
        p.stepSimulation()
        time.sleep(0.002)

def set_gripper(open_pos=0.8):
    time.sleep(0.3)

# === Fixed PyBullet Box Positions (3 slots) ===
# These are the 3 fixed locations in the simulation world.
# CV determines which ArUco ID (= which physical box) occupies each slot.
SLOT_POSITIONS = {
    0: [-0.35, -0.40, 0.025],   # LEFT slot
    1: [-0.25, -0.40, 0.025],   # MIDDLE slot
    2: [-0.15, -0.40, 0.025],   # RIGHT slot
}
SLOT_COLORS = {
    0: [0, 1, 0, 1],   # Green  for left slot
    1: [0, 0, 1, 1],   # Blue   for middle slot
    2: [1, 0, 0, 1],   # Red    for right slot
}
SLOT_LABEL_OFFSET = 0.06

final_drop = [0.28, -0.40, 0.025]

# === Create 3 PyBullet Boxes at Fixed Slots ===
collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025] * 3)

slot_bodies = {}   # slot_index → pybullet body id
for slot_idx, pos in SLOT_POSITIONS.items():
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025] * 3,
                              rgbaColor=SLOT_COLORS[slot_idx])
    body = p.createMultiBody(baseMass=0.5,
                             baseCollisionShapeIndex=collision_shape,
                             baseVisualShapeIndex=vis,
                             basePosition=pos)
    slot_bodies[slot_idx] = body

# Obstacle (static, not touched by CV)
obstacle_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.08, height=0.3)
p.createMultiBody(baseCollisionShapeIndex=obstacle_col,
                  basePosition=[0.10, -0.45, 0.15])

# === ArUco label overlays — updated after CV scan ===
# We keep track of debug text IDs so we can replace them after remapping
label_ids = {}   # slot_index → pybullet debug text id

def refresh_slot_labels(aruco_id_at_slot):
    """
    aruco_id_at_slot: dict  slot_index → aruco_marker_id  (or None if unknown)
    Redraws the 3D labels above each PyBullet box.
    """
    global label_ids
    # Remove old labels
    for lid in label_ids.values():
        p.removeUserDebugItem(lid)
    label_ids = {}

    for slot_idx, pos in SLOT_POSITIONS.items():
        marker_id = aruco_id_at_slot.get(slot_idx)
        label = f"ID {marker_id}" if marker_id is not None else f"Slot {slot_idx + 1}"
        lid = p.addUserDebugText(
            label,
            [pos[0], pos[1] + 0.05, pos[2] + SLOT_LABEL_OFFSET],
            textColorRGB=[0, 0, 0], textSize=1.2
        )
        label_ids[slot_idx] = lid

# Initial labels (before CV scan completes)
refresh_slot_labels({})

# === blocks dict: aruco_marker_id → pybullet body ===
# Default: marker 1 → left slot, marker 2 → middle, marker 3 → right
# This gets REMAPPED by CV scan results.
blocks = {
    1: slot_bodies[0],
    2: slot_bodies[1],
    3: slot_bodies[2],
}

# === ArUco Setup ===
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
ARUCO_PARAMS = aruco.DetectorParameters()

# === Cameras ===
mp_hands = mp.solutions.hands
hands_detector = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils

# Camera 1: Laptop webcam (gesture)
cap_gesture = cv2.VideoCapture(0)
cap_gesture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap_gesture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap_gesture.set(cv2.CAP_PROP_FPS, 30)

# Camera 2: Phone IP camera (ArUco box detection)
# ⚠️  UPDATE THIS IP TO YOUR PHONE'S IP
PHONE_IP = "http://192.168.1.9:8080/video"
cap_vision = cv2.VideoCapture(PHONE_IP)
cap_vision.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap_vision.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap_vision.set(cv2.CAP_PROP_FPS, 15)

# === Pre-create windows so they are resizable and start at a fixed small size ===
cv2.namedWindow("Gesture Camera (Laptop)", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Gesture Camera (Laptop)", 320, 240)

cv2.namedWindow("Vision Camera - ArUco Scan (Phone)", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Vision Camera - ArUco Scan (Phone)", 280, 210)

# === CV Core: ArUco Detection & Slot Remapping ===
def detect_and_remap_boxes(frame):
    """
    Detects ArUco markers (IDs 1, 2, 3) in the phone camera frame.
    Sorts detected markers by their pixel X-centre (left → right).
    Remaps the global `blocks` dict so each marker ID points to the
    correct PyBullet slot body.

    Returns (remapped: bool, annotated_frame, aruco_id_at_slot dict)
    """
    global blocks

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners_list, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT,
                                               parameters=ARUCO_PARAMS)

    if ids is None or len(ids) == 0:
        return False, frame, {}

    # Filter to only IDs 1, 2, 3
    valid = [(int(ids[i][0]), corners_list[i])
             for i in range(len(ids))
             if int(ids[i][0]) in [1, 2, 3]]

    if len(valid) == 0:
        return False, frame, {}

    # Compute pixel X-centre for each detected marker
    def x_centre(corners):
        return float(np.mean(corners[0][:, 0]))

    # Sort by X-centre: leftmost pixel → slot 0 (LEFT), etc.
    valid_sorted = sorted(valid, key=lambda v: x_centre(v[1]))

    # Build new mapping
    new_blocks = {}
    aruco_id_at_slot = {}
    for slot_idx, (marker_id, corners) in enumerate(valid_sorted):
        new_blocks[marker_id] = slot_bodies[slot_idx]
        aruco_id_at_slot[slot_idx] = marker_id

    # Only remap if we detected all 3 markers
    remapped = False
    if len(valid_sorted) == 3:
        blocks = new_blocks
        remapped = True

    # Draw ArUco annotations on frame
    aruco.drawDetectedMarkers(frame, corners_list, ids)
    for slot_idx, (marker_id, corners) in enumerate(valid_sorted):
        cx = int(x_centre(corners))
        cy = int(np.mean(corners[0][:, 1]))
        slot_names = ["LEFT", "MIDDLE", "RIGHT"]
        slot_name = slot_names[slot_idx] if slot_idx < 3 else str(slot_idx)
        cv2.putText(frame, f"ID{marker_id}→{slot_name}",
                    (cx - 30, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return remapped, frame, aruco_id_at_slot

# === Finger Counter ===
def count_fingers(lms):
    """Count raised fingers: index, middle, ring only (landmarks 8, 12, 16)."""
    tips = [8, 12, 16]
    count = 0
    for tip in tips:
        if lms.landmark[tip].y < lms.landmark[tip - 2].y:
            count += 1
    return count

# === Joint Angles (unchanged from original) ===
joint_angles_box = {
    1: {
        'above': [math.radians(-15), math.radians(-90), math.radians(60),
                  math.radians(55), 0.0, 0.0],
        'grip':  [math.radians(-15), math.radians(-90), math.radians(90),
                  math.radians(70), 0.0, 0.0],
    },
    2: {
        'above': [math.radians(-10), math.radians(-90), math.radians(60),
                  math.radians(55), 0.0, 0.0],
        'grip':  [math.radians(-10), math.radians(-90), math.radians(90),
                  math.radians(70), 0.0, 0.0],
    },
    3: {
        'above': [math.radians(-5), math.radians(-90), math.radians(60),
                  math.radians(55), 0.0, 0.0],
        'grip':  [math.radians(-5), math.radians(-90), math.radians(90),
                  math.radians(70), 0.0, 0.0],
    },
}

joint_angles_avoid = [math.radians(30), math.radians(10),
                      math.radians(90), math.radians(70), 0.0, 0.0]
joint_angles_drop  = [math.radians(55), math.radians(-100),
                      math.radians(90), math.radians(90), 0.0, 0.0]

# === State ===
cid = None
current_action = "HOME - waiting for scan"
last_trigger_time = 0
gesture_stable_time = 0
STABLE_DURATION = 1.0
DEBOUNCE = 2.0
commanded_box = None
last_stable_gesture = None

# CV scan state
cv_scan_done = False          # True once all 3 markers detected successfully
aruco_id_at_slot_global = {}  # latest slot mapping for display

frame_counter = 0
VISION_PROCESS_INTERVAL = 3  # process phone frame every N main-loop frames

print("\n" + "=" * 70)
print("  📷 POINT PHONE CAMERA AT ALL 3 BOXES TO SCAN ARUCO MARKERS")
print("  Robot will NOT pick until all 3 markers are detected.")
print("=" * 70 + "\n")

# === Main Loop ===
try:
    while True:
        frame_counter += 1

        # --- Read cameras ---
        ret_gesture, frame_gesture = cap_gesture.read()
        ret_vision,  frame_vision  = cap_vision.read()

        if not ret_gesture:
            print("⚠️  Gesture camera lost")
            break

        frame_gesture = cv2.flip(frame_gesture, 1)

        # --- Phone camera: ArUco detection (every N frames) ---
        if ret_vision and frame_counter % VISION_PROCESS_INTERVAL == 0:
            remapped, frame_vision, aruco_id_at_slot = detect_and_remap_boxes(
                frame_vision)
            if remapped:
                if not cv_scan_done:
                    print("✅ ArUco scan complete! Box mapping:")
                    for slot_idx in range(3):
                        mid = aruco_id_at_slot.get(slot_idx, '?')
                        slot_names = ["LEFT", "MIDDLE", "RIGHT"]
                        print(f"   {slot_names[slot_idx]} slot  →  Marker ID {mid}"
                              f"  →  {slot_names[slot_idx]} PyBullet box")
                    print("   Show 1/2/3 fingers to pick box with that marker ID.\n")
                    cv_scan_done = True
                    current_action = "HOME - ready"
                    refresh_slot_labels(aruco_id_at_slot)
                aruco_id_at_slot_global = aruco_id_at_slot

        # --- Gesture camera: MediaPipe hand detection ---
        rgb = cv2.cvtColor(frame_gesture, cv2.COLOR_BGR2RGB)
        result = hands_detector.process(rgb)
        now = time.time()

        fingers = 0
        is_gesturing = False

        if result.multi_hand_landmarks:
            hand = result.multi_hand_landmarks[0]
            handedness = result.multi_handedness[0].classification[0].label
            if handedness == "Right":
                fingers = count_fingers(hand)
                mp_draw.draw_landmarks(frame_gesture, hand,
                                       mp_hands.HAND_CONNECTIONS)

                if fingers in [1, 2, 3]:
                    is_gesturing = True
                    if last_stable_gesture != fingers:
                        gesture_stable_time = now
                        last_stable_gesture = fingers
                        print(f"👆 {fingers} finger(s) detected — hold for "
                              f"{STABLE_DURATION}s...")
                    commanded_box = fingers
                    box_names = {1: 'Marker-1', 2: 'Marker-2', 3: 'Marker-3'}
                    cv2.putText(frame_gesture,
                                f"Command: Pick {box_names[fingers]}",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 255, 0), 2)

        if not is_gesturing:
            gesture_stable_time = 0
            last_stable_gesture = None

        # --- Trigger: gesture stable + CV scan done ---
        if (is_gesturing and commanded_box in [1, 2, 3]
                and gesture_stable_time > 0):
            time_held = now - gesture_stable_time
            if (time_held >= STABLE_DURATION
                    and now - last_trigger_time >= DEBOUNCE):

                if not cv_scan_done:
                    print("⚠️  ArUco scan not complete yet — point phone at "
                          "all 3 boxes first!")
                elif commanded_box not in blocks:
                    print(f"⚠️  Marker ID {commanded_box} not detected by "
                          f"phone camera — reposition and rescan.")
                else:
                    last_trigger_time = now
                    gesture_stable_time = 0
                    last_stable_gesture = None

                    selected_block = blocks[commanded_box]

                    # Find which slot this marker ended up in (for joint angles)
                    slot_for_marker = None
                    for slot_idx, mid in aruco_id_at_slot_global.items():
                        if mid == commanded_box:
                            slot_for_marker = slot_idx + 1  # 1-indexed
                            break
                    if slot_for_marker is None:
                        slot_for_marker = commanded_box  # fallback

                    print(f"\n✅ EXECUTING: {commanded_box} fingers → "
                          f"Marker ID {commanded_box} → "
                          f"Slot {slot_for_marker}")

                    current_action = f"PICKING Marker {commanded_box}"
                    set_gripper(0.8)
                    move_joints(joint_angles_box[slot_for_marker]['above'])
                    print(f"  → ABOVE box")

                    move_joints(joint_angles_box[slot_for_marker]['grip'])
                    set_gripper(0.0)
                    time.sleep(0.5)
                    cid = p.createConstraint(
                        robot, ee, selected_block, -1,
                        p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
                    print("  → GRIPPING")

                    move_joints(joint_angles_avoid)
                    print("  → AVOIDING obstacle")

                    move_joints(joint_angles_drop)
                    set_gripper(0.8)
                    time.sleep(0.5)
                    if cid is not None:
                        p.removeConstraint(cid)
                        cid = None
                    print("  → RELEASED at drop zone")

                    move_joints(home_joints)
                    # Flush stale camera frames that built up during robot motion
                    for _ in range(15):
                        cap_gesture.read()
                        cap_vision.read()
                    current_action = "HOME - ready"
                    print("  -> HOME\n")

        # --- UI Overlays ---
        scan_status = "CV SCAN OK" if cv_scan_done else "SCAN NEEDED - show boxes to phone"
        cv2.putText(frame_gesture, f"Status: {current_action}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame_gesture, f"Fingers: {fingers}",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(frame_gesture, scan_status,
                    (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if cv_scan_done else (0, 165, 255), 2)

        if gesture_stable_time > 0:
            time_held = now - gesture_stable_time
            progress = min(time_held / STABLE_DURATION, 1.0) * 100
            color = (0, 255, 0) if progress >= 100 else (0, 255, 255)
            cv2.putText(frame_gesture,
                        f"Hold: {time_held:.1f}s ({progress:.0f}%)",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Vision camera window — always show, even if frame not received
        if ret_vision:
            detected_count = len(aruco_id_at_slot_global)
            cv2.putText(frame_vision,
                        f"ArUco detected: {detected_count}/3",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if detected_count == 3 else (0, 165, 255), 2)
            if cv_scan_done:
                for slot_idx, mid in aruco_id_at_slot_global.items():
                    slot_names = ["LEFT", "MID", "RIGHT"]
                    cv2.putText(frame_vision,
                                f"{slot_names[slot_idx]}=ID{mid}",
                                (10, 60 + slot_idx * 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (255, 255, 0), 2)
            cv2.imshow("Vision Camera - ArUco Scan (Phone)", frame_vision)
        else:
            # Phone camera not connected — show a black placeholder so window stays open
            blank = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(blank, "Phone camera not connected", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)
            cv2.putText(blank, f"Expected: {PHONE_IP}", (10, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
            cv2.imshow("Vision Camera - ArUco Scan (Phone)", blank)

        cv2.imshow("Gesture Camera (Laptop)", frame_gesture)

        for _ in range(2):
            p.stepSimulation()

        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    print("\n⚠️  Interrupted by user")

finally:
    cap_gesture.release()
    cap_vision.release()
    cv2.destroyAllWindows()
    p.disconnect()
    print("✅ Cleanup complete")
