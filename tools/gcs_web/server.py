#!/usr/bin/env python3
"""Web GCS dashboard backend: subscribes to PX4's uXRCE-DDS topics (same
pattern as ros2/uav_gcs_bridge/{attitude_listener,kalman_fusion}.py) and
streams a JSON telemetry snapshot to the browser over WebSocket.

Every field in the outgoing JSON is tagged as either real (sourced from the
live STM32 -> hil_bridge.py -> PX4 HITL -> DDS pipeline) or synthetic/derived
- see the "gerçek veri" table in uav_gcs_node/CLAUDE.md. Nothing here invents
data for a channel that has no real wire path (FreeRTOS task health is sent
as unavailable rather than faked).

Prerequisites (run in order, each in its own terminal - same as the rest of
the HITL pipeline, see CLAUDE.md "KOMUTLAR REFERANSI"):
    1. PX4 SITL in HITL mode + tools/hil_bridge.py + MicroXRCEAgent (see
       CLAUDE.md sections on HIL entegrasyonu / ROS2 katmanı).
    2. Gazebo pose-puppet + gimbal camera feed (see CLAUDE.md "Gorsellestirme"):
       gz sim -r tools/gz_puppet/puppet_world.sdf   (or -s for headless)
       tools/attitude_to_gazebo.py (only needed to actually move the puppet -
       the camera publishes frames either way, even at rest)
       ros2 run ros_gz_image image_bridge <CAMERA_GZ_TOPIC below>
    3. This server:
       source /opt/ros/jazzy/setup.bash
       source ~/stm32_ws/ros2_ws/install/setup.bash
       tools/gcs_web/.venv/bin/python tools/gcs_web/server.py

Then open http://127.0.0.1:8765/ in a browser. The OSD/camera panel works
independently of steps 1 - if the video topic isn't bridged the panel just
shows "SİNYAL YOK", and if the HITL pipeline isn't up the OSD shows LINK LOST
(both honest, not faked).
"""

import argparse
import asyncio
import math
import struct
import threading
import time
from pathlib import Path

import cv2
import rclpy
import uvicorn
from cv_bridge import CvBridge
from fastapi import Body, FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pymavlink import mavutil
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image

from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleLocalPosition

STATIC_DIR = Path(__file__).parent / "static"
CESIUM_TOKEN_PLACEHOLDER = "__CESIUM_ION_TOKEN_PLACEHOLDER__"


def _load_cesium_ion_token() -> str:
    """Reads tools/gcs_web/.env (gitignored, never committed - see
    .env.example) for CESIUM_ION_TOKEN. Empty if the file/key is missing;
    the frontend shows an honest "token gerekli" state rather than a fake
    3D terrain view - see index.html's cesium status handling."""
    env_path = Path(__file__).parent / ".env"
    if not env_path.exists():
        return ""
    for line in env_path.read_text().splitlines():
        line = line.strip()
        if line.startswith("CESIUM_ION_TOKEN="):
            return line.split("=", 1)[1].strip()
    return ""


CESIUM_ION_TOKEN = _load_cesium_ion_token()

# Gazebo gimbal camera (tools/gz_puppet/puppet_world.sdf's Sensors plugin),
# bridged to ROS2 by: ros2 run ros_gz_image image_bridge <this topic>
# (creates a sensor_msgs/Image topic of the same name - see CLAUDE.md).
CAMERA_TOPIC = "/world/stm32_puppet_world/model/stm32_puppet/link/camera_link/sensor/camera/image"
VIDEO_FPS = 15
VIDEO_STALE_S = 2.0

# hil_bridge.py's FAKE_LAT_DEG/FAKE_LON_DEG/FAKE_ALT_M (Eskisehir) - the real,
# fixed HIL_GPS fix PX4 is actually holding. Not fabricated: this is what the
# running system genuinely reports, it just never moves (see CLAUDE.md).
HOME_LAT_DEG = 39.7767
HOME_LON_DEG = 30.5206
HOME_ALT_M = 800.0

LINK_TIMEOUT_S = 2.0
ANOMALY_THRESHOLD_DEG = 20.0
BROADCAST_HZ = 15

# PX4's "Normal mode" GCS MAVLink link (see CLAUDE.md's "No connection to the
# GCS" TUZAK) - the same port a real ground control station would connect a
# real vehicle's telemetry radio to. The mission planner uses this for a
# genuine MISSION_ITEM_INT upload, not a demo overlay - see MissionBridge.
#
# udpout (not udpin): PX4 only starts sending to a partner address once it
# has RECEIVED a packet from it (MAV_BROADCAST is off by default - see the
# "MAVLink only on localhost" log line), so we must connect out first: PX4
# then learns our (ephemeral) source port from our own heartbeat and starts
# replying to exactly that. Confirmed empirically (isolated pymavlink probe
# receiving real ATTITUDE/GPS_RAW_INT/etc within ~1s) that this works fine
# ONCE PX4 has actually finished booting its mavlink module - PX4's HITL
# rcS boot script blocks entirely until the first HIL_SENSOR arrives (see
# CLAUDE.md's lockstep TUZAK), so this link isn't live until hil_bridge.py
# is already running and PX4 has connected to it.
MAVLINK_GCS_URL = "udpout:127.0.0.1:18570"
PX4_SYSID = 1
PX4_COMPID = 1  # MAV_COMP_ID_AUTOPILOT1 - PX4 SITL/HITL default
MISSION_ACK_TIMEOUT_S = 5.0

# hil_bridge.py relays the STM32's real NAMED_VALUE_INT task-health telemetry
# (see MAVLink_SendTaskHealth, Core/Src/mavlink_bridge.c) here over local UDP.
TASK_HEALTH_UDP_PORT = 14560
TASK_HEALTH_TIMEOUT_S = 3.0  # WDG_Task sends once/second - 3 missed sends = stale
TASK_HEALTH_NAMES = {
    "hwm_imu": "IMU_Task",
    "hwm_baro": "BARO_Task",
    "hwm_mag": "MAG_Task",
    "hwm_can": "CAN_Task",
    "hwm_mav": "MAVLINK_Task",
    "hwm_wdg": "WDG_Task",
}


def quat_to_euler_deg(q):
    """Same convention as ros2/uav_gcs_bridge/attitude_listener.py::quat_to_euler_deg."""
    w, x, y, z = q
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class AngleKalmanFilter:
    """Copy of ros2/uav_gcs_bridge/kalman_fusion.py::AngleKalmanFilter.

    Duplicated (not imported) so this server has no path dependency on the
    colcon-built uav_gcs_bridge package - it only needs px4_msgs/rclpy.
    """

    def __init__(self, q_angle=0.001, q_bias=0.003, r_measure=0.03):
        self.q_angle = q_angle
        self.q_bias = q_bias
        self.r_measure = r_measure
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]

    def update(self, new_angle, new_rate, dt):
        rate = new_rate - self.bias
        self.angle += dt * rate

        P = self.P
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + self.q_angle)
        P[0][1] -= dt * P[1][1]
        P[1][0] -= dt * P[1][1]
        P[1][1] += self.q_bias * dt

        s = P[0][0] + self.r_measure
        k0 = P[0][0] / s
        k1 = P[1][0] / s

        y = ((new_angle - self.angle + 180.0) % 360.0) - 180.0
        self.angle += k0 * y
        self.bias += k1 * y

        p00, p01 = P[0][0], P[0][1]
        P[0][0] -= k0 * p00
        P[0][1] -= k0 * p01
        P[1][0] -= k1 * p00
        P[1][1] -= k1 * p01

        return self.angle


def encode_can_0x101(ax, ay, az):
    """Re-derives the CAN 0x101 frame bytes from live accel data, using the
    exact encoding documented in CLAUDE.md ("CAN Bus Mesaj Formatı"): float
    -> int16 via x100, big-endian pairs, DLC 8 with 2 trailing zero bytes.
    This is the real accel payload, just re-packed into the frame format the
    firmware would put on the wire in non-loopback mode - not fabricated.
    """
    def i16(v):
        return max(-32768, min(32767, int(round(v * 100))))

    payload = struct.pack(">hhh", i16(ax), i16(ay), i16(az)) + b"\x00\x00"
    return payload.hex(" ").upper()


class TelemetryState:
    """Shared latest-value store, written by the ROS2 callbacks (spin thread)
    and read by the asyncio broadcast loop (main thread) under a lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.start_time = time.time()
        self.last_attitude_t = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.kf_roll = AngleKalmanFilter()
        self.kf_pitch = AngleKalmanFilter()
        self.kf_roll_deg = 0.0
        self.kf_pitch_deg = 0.0
        self._last_sensor_ts_us = None
        self.task_hwm = {}      # NAMED_VALUE_INT name -> (value, last_seen monotonic time)
        self.heap_free = None
        self.heap_free_t = 0.0
        self.frame_lock = threading.Lock()
        self.jpeg_frame = None
        self.jpeg_frame_t = 0.0
        self.cv_bridge = CvBridge()

    def on_image(self, msg: Image):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if not ok:
            return
        with self.frame_lock:
            self.jpeg_frame = buf.tobytes()
            self.jpeg_frame_t = time.time()

    def latest_jpeg(self):
        with self.frame_lock:
            if self.jpeg_frame is None or (time.time() - self.jpeg_frame_t) > VIDEO_STALE_S:
                return None
            return self.jpeg_frame

    def on_task_health(self, name: str, value: int):
        with self.lock:
            now = time.time()
            if name == "heap":
                self.heap_free = value
                self.heap_free_t = now
            else:
                self.task_hwm[name] = (value, now)

    def on_attitude(self, msg: VehicleAttitude):
        with self.lock:
            self.roll_deg, self.pitch_deg, self.yaw_deg = quat_to_euler_deg(msg.q)
            self.last_attitude_t = time.time()

    def on_sensor(self, msg: SensorCombined):
        ax, ay, az = msg.accelerometer_m_s2
        gx, gy, gz = msg.gyro_rad
        with self.lock:
            self.accel = (ax, ay, az)
            self.gyro = (gx, gy, gz)

            if self._last_sensor_ts_us is None:
                self._last_sensor_ts_us = msg.timestamp
                return
            dt = (msg.timestamp - self._last_sensor_ts_us) * 1e-6
            self._last_sensor_ts_us = msg.timestamp
            if dt <= 0.0 or dt > 1.0:
                return

            # Same empirically-fitted sign convention as kalman_fusion.py.
            roll_accel_deg = math.degrees(math.atan2(ay, -az))
            pitch_accel_deg = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
            self.kf_roll_deg = self.kf_roll.update(roll_accel_deg, math.degrees(gx), dt)
            self.kf_pitch_deg = self.kf_pitch.update(pitch_accel_deg, math.degrees(gy), dt)

    def on_local_position(self, msg: VehicleLocalPosition):
        # Present but effectively constant - see CLAUDE.md: hil_bridge.py
        # feeds a static HIL_GPS fix, so PX4's local position never moves.
        pass

    def snapshot(self):
        with self.lock:
            now = time.time()
            link_ok = (now - self.last_attitude_t) < LINK_TIMEOUT_S
            d_roll = abs(((self.kf_roll_deg - self.roll_deg + 180.0) % 360.0) - 180.0)
            d_pitch = abs(((self.kf_pitch_deg - self.pitch_deg + 180.0) % 360.0) - 180.0)
            anomaly = link_ok and (d_roll > ANOMALY_THRESHOLD_DEG or d_pitch > ANOMALY_THRESHOLD_DEG)

            return {
                "t": now,
                "schema_ver": 1,  # bump if the wire payload shape changes - the frontend warns on mismatch
                "uptime_s": now - self.start_time,
                "mode": "HITL",  # this system genuinely runs PX4 SITL in HITL (none_iris) mode
                "link": "OK" if link_ok else "LOST",
                "attitude": {
                    "roll": round(self.roll_deg, 2),
                    "pitch": round(self.pitch_deg, 2),
                    "yaw": round(self.yaw_deg, 2),
                    "real": True,
                },
                "kalman": {
                    "roll": round(self.kf_roll_deg, 2),
                    "pitch": round(self.kf_pitch_deg, 2),
                    "d_roll": round(d_roll, 2),
                    "d_pitch": round(d_pitch, 2),
                    "anomaly": anomaly,
                    "threshold": ANOMALY_THRESHOLD_DEG,
                    "real": True,
                },
                "sensor": {
                    "accel": [round(float(v), 3) for v in self.accel],
                    "gyro": [round(float(v), 4) for v in self.gyro],
                    "real": True,
                },
                "position": {
                    "lat": HOME_LAT_DEG,
                    "lon": HOME_LON_DEG,
                    "alt": HOME_ALT_M,
                    "heading": round(self.yaw_deg % 360.0, 1),
                    "static": True,
                    "real": True,
                },
                "battery": {
                    # No ADC/VBAT sense on the STM32 (see CLAUDE.md) - purely
                    # illustrative so the panel isn't empty, tagged sim=True
                    # end-to-end so the frontend can badge it.
                    "voltage": 11.1,
                    "percent": 76,
                    "sim": True,
                },
                "can": {
                    "frames": [
                        {
                            "id": "0x101",
                            "label": "accel",
                            "bytes": encode_can_0x101(*self.accel),
                            "pending": False,
                        },
                        {"id": "0x102", "label": "baro", "bytes": None, "pending": True},
                        {"id": "0x103", "label": "mag", "bytes": None, "pending": True},
                    ],
                    "note": "loopback-only on STM32, re-encoded here from live accel; baro/mag are firmware TODO stubs",
                },
                "freertos": self._freertos_snapshot(now),
                "video": {"available": self.latest_jpeg() is not None, "real": True},
            }

    def _freertos_snapshot(self, now):
        # Called with self.lock already held (from snapshot()).
        tasks = []
        freshest = 0.0
        for key, label in TASK_HEALTH_NAMES.items():
            entry = self.task_hwm.get(key)
            if entry is None:
                continue
            value, seen_t = entry
            freshest = max(freshest, seen_t)
            tasks.append({"name": label, "hwm": value, "fresh": (now - seen_t) < TASK_HEALTH_TIMEOUT_S})

        heap_fresh = self.heap_free is not None and (now - self.heap_free_t) < TASK_HEALTH_TIMEOUT_S
        available = bool(tasks) and (now - freshest) < TASK_HEALTH_TIMEOUT_S

        if not available:
            return {
                "available": False,
                "reason": "no wire path from WDG_Task to PC (DBG_PRINT is debug-UART-only)"
                          if not tasks else "STM32'den task-health verisi kesildi (son görülen > "
                                            f"{TASK_HEALTH_TIMEOUT_S:.0f}s önce)",
            }
        return {
            "available": True,
            "real": True,
            "tasks": tasks,
            "heap_free": self.heap_free if heap_fresh else None,
        }


class MissionBridge:
    """Real MAVLink mission upload to PX4's GCS link (127.0.0.1:18570), using
    the standard MISSION_COUNT / MISSION_REQUEST_INT / MISSION_ITEM_INT /
    MISSION_ACK handshake - the identical protocol a real GCS speaks to real
    autopilot hardware, and PX4 doesn't distinguish HITL from a physical
    vehicle here. This is why the dashboard's mission panel can honestly
    drop the "DEMO" label the map's illustrative waypoint route still
    carries - contrast with the ARM/DISARM/HOLD/RTL buttons in index.html,
    which stay intentionally unwired (demoCmd()) since arming isn't in
    scope here.

    One long-lived connection is opened at startup and reused for the life
    of the process - CLAUDE.md documents the GCS link as "yapışkan" (PX4
    can be slow/inconsistent accepting a *new* short-lived connection), so
    reconnecting per-request would be fragile.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.master = mavutil.mavlink_connection(
            MAVLINK_GCS_URL, source_system=255, source_component=190)
        self.connected = False
        self.last_ack = None       # "ACCEPTED" | "CLEARED" | "<error text>" | None
        self.px4_wp_count = None   # count PX4 last ACKed (from our own upload/clear, not a separate read-back)
        self.uploading = False
        self.waypoints = []        # last successfully uploaded [{lat, lon, alt}, ...]

    def status(self):
        with self.lock:
            return {
                "connected": self.connected,
                "uploading": self.uploading,
                "last_ack": self.last_ack,
                "px4_wp_count": self.px4_wp_count,
                "waypoints": self.waypoints,
            }

    def run_heartbeat_loop(self):
        """Background thread: sends our own HEARTBEAT (MAV_TYPE_GCS) at ~3Hz -
        PX4 expects to see one from a GCS link (see CLAUDE.md's arm-test
        TUZAK), and only starts replying to OUR address once it has received
        at least one packet from us (MAV_BROADCAST is off by default - see
        CLAUDE.md's "connected" TUZAK). Deliberately does NOT poll mission
        count on a timer - PX4's waypoint manager (WPM) is a strict single-
        transaction state machine, and a MISSION_REQUEST_LIST outside of an
        upload/clear call left it stuck reporting "WPM: IGN ...: Busy" (see
        CLAUDE.md TUZAK). px4_wp_count only ever comes from our own
        upload()/clear() ACKs."""
        while True:
            with self.lock:
                if not self.uploading:
                    try:
                        self.master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                        hb = self.master.recv_match(type="HEARTBEAT", blocking=False)
                        if hb is not None:
                            self.connected = True
                    except Exception:
                        pass
            time.sleep(0.3)

    def upload(self, waypoints):
        """waypoints: [{"lat":.., "lon":.., "alt":..}, ...]. Blocking - call
        via asyncio.to_thread from the FastAPI handler, never from the event
        loop directly."""
        with self.lock:
            self.uploading = True
            try:
                self.master.mav.mission_count_send(PX4_SYSID, PX4_COMPID, len(waypoints), 0)
                for _ in range(len(waypoints)):
                    req = self.master.recv_match(
                        type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
                        blocking=True, timeout=MISSION_ACK_TIMEOUT_S)
                    if req is None:
                        self.last_ack = "TIMEOUT (MISSION_REQUEST alınamadı)"
                        return False
                    wp = waypoints[req.seq]
                    self.master.mav.mission_item_int_send(
                        PX4_SYSID, PX4_COMPID, req.seq,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 1,
                        0, 0, 0, 0,
                        int(round(wp["lat"] * 1e7)), int(round(wp["lon"] * 1e7)), float(wp["alt"]),
                        0)
                ack = self.master.recv_match(type="MISSION_ACK", blocking=True, timeout=MISSION_ACK_TIMEOUT_S)
                if ack is None:
                    self.last_ack = "TIMEOUT (MISSION_ACK alınamadı)"
                    return False
                if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self.last_ack = "ACCEPTED"
                    self.waypoints = waypoints
                    # PX4's own ACK already confirms exactly this count was
                    # stored - a follow-up MISSION_REQUEST_LIST read-back is
                    # NOT queried here on purpose: it left PX4's waypoint
                    # manager (WPM) stuck reporting later commands "Busy"
                    # (see CLAUDE.md TUZAK) since WPM is a strict single-
                    # transaction state machine.
                    self.px4_wp_count = len(waypoints)
                    return True
                self.last_ack = f"REJECTED (type={ack.type})"
                return False
            except Exception as e:
                self.last_ack = f"ERROR: {e}"
                return False
            finally:
                self.uploading = False

    def clear(self):
        with self.lock:
            self.uploading = True
            try:
                self.master.mav.mission_clear_all_send(PX4_SYSID, PX4_COMPID, 0)
                ack = self.master.recv_match(type="MISSION_ACK", blocking=True, timeout=MISSION_ACK_TIMEOUT_S)
                ok = ack is not None and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
                self.last_ack = "CLEARED" if ok else "CLEAR TIMEOUT/REJECTED"
                if ok:
                    self.waypoints = []
                    self.px4_wp_count = 0
                return ok
            except Exception as e:
                self.last_ack = f"ERROR: {e}"
                return False
            finally:
                self.uploading = False


class GcsWebNode(Node):
    def __init__(self, state: TelemetryState):
        super().__init__("gcs_web_bridge")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", state.on_attitude, qos)
        self.create_subscription(SensorCombined, "/fmu/out/sensor_combined", state.on_sensor, qos)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", state.on_local_position, qos)
        self.create_subscription(Image, CAMERA_TOPIC, state.on_image, qos_profile_sensor_data)
        self.get_logger().info("gcs_web_bridge started, PX4 uXRCE-DDS topiclerini bekliyor...")


def run_ros_spin(node):
    rclpy.spin(node)


def run_task_health_listener(state: TelemetryState, port: int):
    """Reads the NAMED_VALUE_INT task-health stream hil_bridge.py relays over
    local UDP (see its --task-health-port). Independent of the ROS2/DDS path -
    this is raw MAVLink straight from the STM32, just forwarded once."""
    conn = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
    while True:
        msg = conn.recv_match(type="NAMED_VALUE_INT", blocking=True)
        if msg is None:
            continue
        name = msg.name.rstrip("\x00") if isinstance(msg.name, str) else msg.name
        state.on_task_health(name, msg.value)


app = FastAPI()
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
state = TelemetryState()
mission_bridge = MissionBridge()


@app.get("/")
async def index():
    html = (STATIC_DIR / "index.html").read_text()
    html = html.replace(CESIUM_TOKEN_PLACEHOLDER, CESIUM_ION_TOKEN)
    return HTMLResponse(html)


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            payload = state.snapshot()
            payload["mission"] = mission_bridge.status()
            await ws.send_json(payload)
            await asyncio.sleep(1.0 / BROADCAST_HZ)
    except WebSocketDisconnect:
        pass


@app.post("/api/mission/upload")
async def mission_upload(payload: dict = Body(...)):
    waypoints = payload.get("waypoints") or []
    if not waypoints:
        return {"ok": False, "error": "waypoint listesi boş"}
    ok = await asyncio.to_thread(mission_bridge.upload, waypoints)
    return {"ok": ok, "status": mission_bridge.status()}


@app.post("/api/mission/clear")
async def mission_clear():
    ok = await asyncio.to_thread(mission_bridge.clear)
    return {"ok": ok, "status": mission_bridge.status()}


async def _mjpeg_frames():
    period = 1.0 / VIDEO_FPS
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    while True:
        frame = state.latest_jpeg()
        if frame is not None:
            yield boundary + frame + b"\r\n"
        await asyncio.sleep(period)


@app.get("/video/stream.mjpg")
async def video_stream():
    return StreamingResponse(_mjpeg_frames(), media_type="multipart/x-mixed-replace; boundary=frame")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    rclpy.init()
    node = GcsWebNode(state)
    spin_thread = threading.Thread(target=run_ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    health_thread = threading.Thread(
        target=run_task_health_listener, args=(state, TASK_HEALTH_UDP_PORT), daemon=True)
    health_thread.start()

    mission_thread = threading.Thread(target=mission_bridge.run_heartbeat_loop, daemon=True)
    mission_thread.start()

    print(f"[*] GCS dashboard: http://{args.host}:{args.port}/")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
