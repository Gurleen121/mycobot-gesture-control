#!/usr/bin/env python3
import cv2
import mediapipe as mp
import numpy as np
import socket
import struct
import time
import json
from collections import defaultdict, deque
# === MULTICAST SETUP ===
MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5007
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
# === CAMERA SETUP ===
PHONE_IP = "http://172.29.2.110:8080/video"
cap_vision = cv2.VideoCapture(PHONE_IP)
cap_gesture = cv2.VideoCapture(0)
cap_vision.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap_vision.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap_vision.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap_gesture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap_gesture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
# Warmup
for _ in range(50):
    cap_vision.read()
    cap_gesture.read()
print("📹 Cameras warmed up. Starting 30s calibration... Keep vision cam steady on boxes (30-50cm away)!")
# === MEDIAPIPE HAND DETECTION ===
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.6,
                       min_tracking_confidence=0.4)
def count_fingers(lms):
    tips = [4, 8, 12, 16, 20]
    count = 0
    if lms.landmark[4].x < lms.landmark[3].x:
        count += 1
    for tip in [8, 12, 16, 20]:
        if lms.landmark[tip].y < lms.landmark[tip-2].y:
            count += 1
    return count
# === CALIBRATION TRACKING ===
box_buffers = defaultdict(lambda: deque(maxlen=15))  # Longer for shake smoothing
POS_TOLERANCE = 50  # Wider for hand-hold jitter
MIN_POS_COUNT = 3  # Need ≥3 unique pos for good calib
def get_pos_hash(pos):
    x, y = pos
    return f"{int(x)//POS_TOLERANCE}_{int(y)//POS_TOLERANCE}"
def calibrate_boxes(calib_time=30):
    print(f"🔧 Calibrating for {calib_time}s... Move away after. Steady hand!")
    start_time = time.time()
    pos_to_lines = {}
    while time.time() - start_time < calib_time:
        ret, frame = cap_vision.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        edges = cv2.Canny(gray, 40, 150)  # Lower low thresh for faint lines
        kernel = np.ones((2,2), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (128,128,128), 1)
       
        current_detections = {}
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < 400: continue
            x, y, w, h = cv2.boundingRect(cnt)
            if w < 15 or h < 15: continue
            aspect = w / h
            if not (0.3 < aspect < 3.0): continue
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            if solidity < 0.4: continue
           
            roi_edges = edges[y:y+h, x:x+w]
            lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180, threshold=6, minLineLength=10, maxLineGap=6)  # Looser for III
            vertical_lines = []
            if lines is not None:
                for line in lines:
                    lx1, ly1, lx2, ly2 = line[0]
                    if abs(lx2 - lx1) < 8 and abs(ly2 - ly1) > 12 and ly1 < ly2:
                        vertical_lines.append(line)
           
            num_lines = len(vertical_lines)
            if num_lines < 1: continue
            pos = (x + w//2, y + h//2)
            pos_hash = get_pos_hash(pos)
            current_detections[pos_hash] = num_lines
            buffer = box_buffers[pos_hash]
            buffer.append(num_lines)
           
            # Draw
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255,255,255), 2)
            for line in vertical_lines:
                lx1, ly1, lx2, ly2 = line[0]
                cv2.line(frame, (x + lx1, y + ly1), (x + lx2, y + ly2), (255,255,255), 2)
            avg = sum(buffer) / len(buffer)
            cv2.putText(frame, f"{pos_hash} {num_lines} ({avg:.1f})", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
       
        # Avg update
        for pos_hash in current_detections:
            avg = sum(box_buffers[pos_hash]) / len(box_buffers[pos_hash])
            pos_to_lines[pos_hash] = round(avg)
            print(f"📏 Pos {pos_hash}: {current_detections[pos_hash]} lines (running avg {avg:.1f})")
        
        # Debug: Print unique pos count
        unique_pos = len(current_detections)
        cv2.putText(frame, f"Unique pos: {unique_pos}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.imshow("Vision (Calib - White boxes/lines)", frame)
        if cv2.waitKey(1) == 27: break
   
    cv2.destroyWindow("Vision (Calib - White boxes/lines)")
    print(f"📊 Final pos found: {len(pos_to_lines)} (need ≥{MIN_POS_COUNT})")
    if len(pos_to_lines) < MIN_POS_COUNT:
        print("⚠️ Low pos count – rerun calib steadier or manual edit json!")
    # Map
    box_map = {}
    for avg_lines in sorted(set(pos_to_lines.values())):
        if avg_lines in [1,2,3]:
            box_map[avg_lines] = avg_lines
    print(f"✅ Calib complete: {box_map}")
    # Save local (for backup)
    try:
        with open('box_map.json', 'w') as f:
            json.dump(box_map, f)
        print("💾 Saved local box_map.json")
    except Exception as e:
        print(f"⚠️ Local save error: {e}")
    # Send calib signal (0)
    try:
        calib_msg = struct.pack('B', 0)
        sock.sendto(calib_msg, (MCAST_GRP, MCAST_PORT))
        print("📤 Sent calib signal (0)")
        time.sleep(0.5) # Brief pause
        # Send map data (cmd=4 + JSON bytes)
        map_json = json.dumps(box_map).encode('utf-8')
        map_msg = struct.pack('B', 4) + map_json # Prepend cmd 4
        sock.sendto(map_msg, (MCAST_GRP, MCAST_PORT))
        print(f"📤 Sent map data (cmd=4, size={len(map_json)} bytes): {box_map}")
    except Exception as e:
        print(f"❌ Send error: {e}")
    return box_map
# === RUNTIME ===
def runtime_mode(box_map):
    last_sent = 0
    send_debounce = 5.0  # Increased to 5s for sequence time
    last_fingers = 0  # Track last sent gesture to avoid repeats
    stable_start = 0  # Time when current fingers started
    STABLE_TIME = 5.0  # Hold 5s for confirm (adjust to 7.0 if wanted)
    while True:
        ret_g, frame_g = cap_gesture.read()
        if not ret_g:
            time.sleep(0.1)
            continue
        frame_g = cv2.flip(frame_g, 1)
        # Hand
        fingers = 0
        rgb = cv2.cvtColor(frame_g, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)
        handedness = "None"
        if result.multi_hand_landmarks:
            hand = result.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame_g, hand, mp_hands.HAND_CONNECTIONS)
            handedness = result.multi_handedness[0].classification[0].label if result.multi_handedness else "Unknown"
            if handedness == "Right" or handedness == "Unknown":
                fingers = count_fingers(hand)
                print(f"✋ Fingers: {fingers} (hand: {handedness})")
        # Stability logic
        if fingers != last_fingers:
            stable_start = time.time()  # Reset timer on change
            print(f"🔄 Gesture change {last_fingers}→{fingers} – hold {STABLE_TIME}s to confirm")
            last_fingers = fingers
        elif fingers in [1, 2, 3] and fingers in box_map:
            stable_duration = time.time() - stable_start
            if stable_duration >= STABLE_TIME and time.time() - last_sent > send_debounce:
                try:
                    msg = struct.pack('B', fingers)
                    sent = sock.sendto(msg, (MCAST_GRP, MCAST_PORT))
                    if sent > 0:
                        print(f"🎯 Stable {fingers} confirmed ({stable_duration:.1f}s) – Sent: Pick Box {fingers}!")
                        last_sent = time.time()
                        stable_start = 0  # Reset after send
                    else:
                        print("⚠️ Send failed (0 bytes)")
                except Exception as e:
                    print(f"❌ Send error: {e}")
            else:
                print(f"⏳ Stable {fingers}: {stable_duration:.1f}s / {STABLE_TIME}s needed")
        elif fingers == 0:
            if last_fingers != 0:
                print(f"👋 Gesture cleared (was {last_fingers}) – ready")
                last_fingers = 0
                stable_start = 0
        # UI
        cv2.putText(frame_g, f"Fingers: {fingers}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(frame_g, f"Hand: {handedness}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)
        cv2.putText(frame_g, f"Map: {box_map}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        cv2.putText(frame_g, f"Last: {last_fingers}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
        stable_text = f"Stable: {last_fingers} ({time.time() - stable_start:.1f}s)" if last_fingers > 0 else "Ready"
        cv2.putText(frame_g, stable_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        cv2.imshow("Gesture (ESC to quit)", frame_g)
        if cv2.waitKey(1) == 27:
            break
# === MAIN ===
box_map = calibrate_boxes(30)
runtime_mode(box_map)
cap_gesture.release()
cap_vision.release()
cv2.destroyAllWindows()
sock.close()
print("👋 Talker shutdown.")
