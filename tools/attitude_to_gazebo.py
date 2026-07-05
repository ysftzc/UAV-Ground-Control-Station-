#!/usr/bin/env python3
"""Visualization bridge: read PX4's live ATTITUDE estimate (computed from
real STM32 sensor data via tools/hil_bridge.py) and teleport a static model's
pose in a plain Gazebo world (tools/gz_puppet/puppet_world.sdf) to match it.

This is a pose puppet, not a physics simulation: PX4 SITL runs headless in
none_iris (HITL) mode and never touches this Gazebo world directly - gz_x500
(full physics) and none_iris (HITL) can't run in the same PX4 instance, so
visualization is done by directly setting the model's world pose via the
Gazebo "set_pose" service each time a new ATTITUDE arrives.

Prerequisites (run these first, each in its own terminal):
    1. PX4 SITL in HITL mode:
       cd ~/stm32_ws/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/simulator_mavlink
       PX4_SYS_AUTOSTART=10016 ../../../../bin/px4 -d
    2. The sensor bridge:
       .venv/bin/python tools/hil_bridge.py
    3. The puppet world:
       GZ_SIM_RESOURCE_PATH=~/stm32_ws/PX4-Autopilot/Tools/simulation/gz/models \\
           gz sim -r tools/gz_puppet/puppet_world.sdf

Usage:
    .venv/bin/python tools/attitude_to_gazebo.py [--gcs-port 18570] [--model stm32_puppet]
                                                   [--world stm32_puppet_world]
"""

import argparse
import math
import sys
import threading
import time

from pymavlink import mavutil

import gz.transport13 as transport
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

PUPPET_HEIGHT_M = 1.5


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gcs-host", default="127.0.0.1")
    parser.add_argument("--gcs-port", type=int, default=18570)
    parser.add_argument("--model", default="stm32_puppet")
    parser.add_argument("--world", default="stm32_puppet_world")
    args = parser.parse_args()

    print(f"[*] PX4 GCS baglantisi: udpout:{args.gcs_host}:{args.gcs_port}")
    m = mavutil.mavlink_connection(f"udpout:{args.gcs_host}:{args.gcs_port}",
                                    source_system=255, source_component=190)

    stop = threading.Event()

    def heartbeat_loop():
        while not stop.is_set():
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1)

    threading.Thread(target=heartbeat_loop, daemon=True).start()

    m.wait_heartbeat(timeout=10)
    print("[*] PX4 baglandi, ATTITUDE bekleniyor...")

    node = transport.Node()
    service = f"/world/{args.world}/set_pose"

    count = 0
    try:
        while True:
            msg = m.recv_match(type="ATTITUDE", blocking=True, timeout=2)
            if msg is None:
                print("[!] 2sn icinde ATTITUDE gelmedi")
                continue

            qx, qy, qz, qw = euler_to_quat(msg.roll, msg.pitch, msg.yaw)

            req = Pose()
            req.name = args.model
            req.position.x = 0.0
            req.position.y = 0.0
            req.position.z = PUPPET_HEIGHT_M
            req.orientation.x = qx
            req.orientation.y = qy
            req.orientation.z = qz
            req.orientation.w = qw

            result, response = node.request(service, req, Pose, Boolean, 200)
            count += 1

            if count % 50 == 0:
                ok = response.data if result else False
                print(f"[*] {count} pose guncellemesi gonderildi "
                      f"(roll={msg.roll:+.2f} pitch={msg.pitch:+.2f} yaw={msg.yaw:+.2f}, "
                      f"set_pose basarili={ok})")

    except KeyboardInterrupt:
        stop.set()
        print(f"\n[*] Durduruldu. Toplam {count} pose guncellemesi gonderildi.")
        sys.exit(0)


if __name__ == "__main__":
    main()
