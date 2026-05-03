#!/usr/bin/env python3
import socket
import struct
import time
import sys
import json
from pymycobot.mycobot import MyCobot
# === Multicast Setup ===
MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5007
# === MyCobot Setup (with baud fallback) ===
bauds = [1000000, 115200]
mc = None
for baud in bauds:
    try:
        mc = MyCobot('/dev/ttyAMA0', baud)
        mc.set_gripper_state(0, 50)
        time.sleep(1.5)
        print(f"✅ MyCobot connected on /dev/ttyAMA0 at {baud} baud")
        break
    except Exception as e:
        print(f"⚠️ Baud {baud} failed: {e}")
        if baud == bauds[-1]:
            print("❌ All bauds failed – check cable/port!")
            sys.exit(1)
# === Joint Angles for Each Box ===
joint_angles = {
    1: { # Box I
        'pick': [-21.56, -43.33, -65.21, 11.86, 4.04, -61.34],
        'drop': [74.97, 3.16, -133.15, 30.23, -1.93, 30.23]
    },
    2: { # Box II
        'pick': [-5.53, -33.13, -84.37, 13.35, 6.85, -51.24],
        'drop': [56.16, -27.59, -105.2, 35.41, -0.43, 8.52]
    },
    3: { # Box III
        'pick': [16.17, -19.42, -115.83, 43.94, 1.66, -33.22],
        'drop': [46.31, -43.15, -78.66, 29.61, -6.76, 6.06]
    }
}
# === COMMON WAYPOINT (degrees) ===
WAYPOINT = [-0.17, -2.63, 6.32, -16.34, -0.7, 136.93]
# === Sequence busy lock ===
sequence_busy = False
sequence_start_time = 0
SEQUENCE_TIMEOUT = 25.0  # Max sequence time + buffer
# === Function to wait for arm to settle ===
def wait_for_settle(mc, target_angles, tolerance=2.0, timeout=10.0):
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            current_angles = mc.get_angles()
            diff = max(abs(current_angles[i] - target_angles[i]) for i in range(6))
            if diff < tolerance:
                print(f"✅ Arm settled (diff: {diff:.2f}°)")
                return True
        except Exception as e:
            print(f"⚠️ Settle read error: {e}")
        time.sleep(0.2)
    print("⚠️ Timeout waiting for settle")
    return False
# === Retry wrapper for serial commands ===
def safe_send_angles(mc, angles, speed, max_retries=3):
    for attempt in range(max_retries):
        try:
            mc.send_angles(angles, speed)
            print(f"✅ Sent angles (try {attempt+1})")
            return True
        except Exception as e:
            print(f"⚠️ Send angles failed (try {attempt+1}): {e}")
            if attempt < max_retries - 1:
                time.sleep(0.5)
            else:
                print("❌ Max retries failed – skipping move")
                return False
    return False
def safe_gripper(mc, state, speed=50):
    for attempt in range(3):
        try:
            mc.set_gripper_state(state, speed)
            print(f"✅ Gripper {'closed' if state else 'open'} (try {attempt+1})")
            return True
        except Exception as e:
            print(f"⚠️ Gripper failed (try {attempt+1}): {e}")
            if attempt < 2:
                time.sleep(0.5)
    return False
# === Multicast Listener ===
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', MCAST_PORT))
# IP auto-detect
import subprocess
local_ip = '0.0.0.0'
try:
    result = subprocess.run(['ip', 'route', 'get', MCAST_GRP], capture_output=True, text=True, check=True)
    lines = result.stdout.strip().split('\n')
    for line in lines:
        if 'src' in line:
            local_ip = line.split('src ')[1].split()[0]
            break
    print(f"🌐 Using multicast interface IP: {local_ip}")
except Exception as e:
    print(f"⚠️ Could not auto-detect multicast IP: {e}. Using 0.0.0.0")
group = socket.inet_aton(MCAST_GRP)
mreq = struct.pack('4s4s', group, socket.inet_aton(local_ip))
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
# Calib state
calib_received = False
box_map = {1:1, 2:2, 3:3}  # Default
print("👂 Listener ready. Waiting for calib signal (0), map (4), then commands (1-3)...")
def execute_sequence(box_num):
    global sequence_busy, sequence_start_time
    sequence_busy = True
    sequence_start_time = time.time()
    print(f"🚀 Starting sequence for Box {box_num} (busy until {SEQUENCE_TIMEOUT}s)")
    # 1. PICK
    print("→ Moving to PICK position")
    pick_angles = joint_angles[box_num]['pick']
    if safe_send_angles(mc, pick_angles, 50):
        wait_for_settle(mc, pick_angles)
        time.sleep(0.5)
    else:
        print("⚠️ Skipping PICK")
        sequence_busy = False
        return
    # 2. CLOSE
    print("→ Closing gripper")
    if safe_gripper(mc, 1):
        time.sleep(2.0)
    # 3. WAYPOINT
    print("→ Moving to WAYPOINT")
    if safe_send_angles(mc, WAYPOINT, 50):
        wait_for_settle(mc, WAYPOINT)
    # 4. DROP
    print("→ Moving to DROP position")
    drop_angles = joint_angles[box_num]['drop']
    if safe_send_angles(mc, drop_angles, 50):
        wait_for_settle(mc, drop_angles)
    # 5. OPEN
    print("→ Opening gripper")
    if safe_gripper(mc, 0):
        time.sleep(1.0)
    # 6. Return
    print("→ Returning to WAYPOINT")
    if safe_send_angles(mc, WAYPOINT, 50):
        wait_for_settle(mc, WAYPOINT)
    print(f"✅ Box {box_num} sequence complete!")
    # End busy
    sequence_busy = False
    print("🆓 Sequence done – ready for next!")
try:
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"📥 Raw data from {addr}: {data} (hex: {data.hex()})")
        if len(data) < 1: continue
        cmd = struct.unpack('B', data[:1])[0]
        print(f"📦 Parsed command: {cmd}")
        if cmd == 0:  # Calib signal
            calib_received = True
            print("✅ Calib signal received – waiting for map...")
            continue
        if cmd == 4:  # Map data
            try:
                map_json = data[1:].decode('utf-8')
                loaded_map = json.loads(map_json)
                box_map = {int(k): int(v) for k, v in loaded_map.items()}
                print(f"✅ Map received + loaded: {box_map}")
                # Optional save
                with open('box_map.json', 'w') as f:
                    json.dump(box_map, f)
                print("💾 Saved box_map.json locally")
            except Exception as e:
                print(f"❌ Map unpack error: {e} – using default")
            continue
        if not calib_received:
            print("❌ Waiting for calib (0) + map (4) before commands!")
            continue
        if cmd in [1, 2, 3]:
            if sequence_busy:
                elapsed = time.time() - sequence_start_time
                remain = SEQUENCE_TIMEOUT - elapsed
                print(f"⏳ Busy with sequence ({elapsed:.1f}s elapsed, {remain:.1f}s remain) – ignore cmd {cmd}")
                continue
            box_num = box_map.get(cmd, cmd)
            execute_sequence(box_num)
        else:
            print(f"❓ Invalid: {cmd}")
except KeyboardInterrupt:
    print("\n🛑 Stopped by user")
finally:
    if mc:
        try:
            mc.set_gripper_state(0, 50)
        except: pass
    sock.close()
    print("🔌 Listener shutdown complete.")
