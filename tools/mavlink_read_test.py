#!/usr/bin/env python3
"""Diagnostic tool: read HEARTBEAT + HIL_SENSOR from the STM32 GCS node over UART.

Usage:
    .venv/bin/python tools/mavlink_read_test.py [--port /dev/ttyUSB0] [--baud 115200]
"""

import argparse
import sys

from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print(f"[*] {args.port} baglaniliyor ({args.baud} baud)...")
    conn = mavutil.mavlink_connection(args.port, baud=args.baud)

    print("[*] MAVLink akisi bekleniyor (Ctrl+C ile cik)...")
    heartbeat_count = 0
    hil_sensor_count = 0

    try:
        while True:
            msg = conn.recv_match(blocking=True, timeout=5)
            if msg is None:
                print("[!] 5sn icinde mesaj gelmedi - baglanti/UART kontrol et")
                continue

            if msg.get_type() == "BAD_DATA":
                continue

            if msg.get_type() == "HEARTBEAT":
                heartbeat_count += 1
                print(f"[HEARTBEAT #{heartbeat_count}] sysid:{msg.get_srcSystem()} "
                      f"compid:{msg.get_srcComponent()} type:{msg.type} "
                      f"autopilot:{msg.autopilot} state:{msg.system_status}")

            elif msg.get_type() == "HIL_SENSOR":
                hil_sensor_count += 1
                print(f"[HIL_SENSOR #{hil_sensor_count}] "
                      f"acc:({msg.xacc:.2f},{msg.yacc:.2f},{msg.zacc:.2f}) m/s^2  "
                      f"gyro:({msg.xgyro:.3f},{msg.ygyro:.3f},{msg.zgyro:.3f}) rad/s  "
                      f"mag:({msg.xmag:.3f},{msg.ymag:.3f},{msg.zmag:.3f}) G  "
                      f"press:{msg.abs_pressure:.1f} hPa  temp:{msg.temperature:.1f} C  "
                      f"alt:{msg.pressure_alt:.1f} m")

    except KeyboardInterrupt:
        print(f"\n[*] Durduruldu. Toplam: {heartbeat_count} HEARTBEAT, "
              f"{hil_sensor_count} HIL_SENSOR")
        sys.exit(0)


if __name__ == "__main__":
    main()
