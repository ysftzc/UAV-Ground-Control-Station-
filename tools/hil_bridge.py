#!/usr/bin/env python3
"""HIL bridge: forward STM32 HIL_SENSOR (real IMU/baro/mag over UART) into
PX4 SITL's simulator_mavlink TCP link, so PX4's actual EKF2/attitude estimator
runs on real hardware sensor data instead of simulated physics.

PX4 side: run SITL in HITL mode first (no gz/jmavsim visual sim):
    cd ~/stm32_ws/PX4-Autopilot
    PX4_SIM_MODEL=none_iris make px4_sitl none_iris

That starts `simulator_mavlink` listening as a TCP server on 127.0.0.1:4560.

Usage:
    .venv/bin/python tools/hil_bridge.py [--serial-port /dev/ttyUSB0] [--baud 115200]
                                          [--px4-host 127.0.0.1] [--px4-port 4560]
"""

import argparse
import sys
import time

from pymavlink import mavutil

# No real GPS on the board yet: feed PX4 a fixed static fix so EKF2 can get a
# global position estimate. Eskisehir, Turkey, roughly at sea level.
FAKE_LAT_DEG = 39.7767
FAKE_LON_DEG = 30.5206
FAKE_ALT_M = 800.0
GPS_UPDATE_PERIOD_S = 0.2  # 5 Hz, well within what HIL_GPS needs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--px4-host", default="127.0.0.1")
    parser.add_argument("--px4-port", type=int, default=4560)
    args = parser.parse_args()

    print(f"[*] STM32 seri baglantisi: {args.serial_port} @ {args.baud} baud")
    stm32 = mavutil.mavlink_connection(args.serial_port, baud=args.baud)

    # PX4'un simulator_mavlink modulu TCP CLIENT'tir (disariya connect() eder,
    # "Waiting for simulator to accept connection" mesaji yaniltici) - o yuzden
    # biz burada TCP SERVER olup PX4'un baglanmasini bekliyoruz.
    print(f"[*] PX4 SITL baglantisi bekleniyor: tcpin:{args.px4_host}:{args.px4_port}")
    px4 = mavutil.mavlink_connection(f"tcpin:{args.px4_host}:{args.px4_port}", source_system=1,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
    print("[*] PX4 baglandi.")

    hil_sensor_count = 0
    last_gps_sent = 0.0
    t_start = time.time()

    try:
        while True:
            msg = stm32.recv_match(blocking=True, timeout=2)
            if msg is None:
                print("[!] STM32'den 2sn icinde veri gelmedi")
                continue

            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                continue

            if mtype == "HIL_SENSOR":
                hil_sensor_count += 1
                time_usec = int((time.time() - t_start) * 1e6)

                px4.mav.hil_sensor_send(
                    time_usec,
                    msg.xacc, msg.yacc, msg.zacc,
                    msg.xgyro, msg.ygyro, msg.zgyro,
                    msg.xmag, msg.ymag, msg.zmag,
                    msg.abs_pressure, msg.diff_pressure, msg.pressure_alt,
                    msg.temperature,
                    msg.fields_updated,
                    0,  # id: primary IMU, drives PX4 lockstep clock
                )

                if hil_sensor_count % 100 == 0:
                    print(f"[*] {hil_sensor_count} HIL_SENSOR PX4'e iletildi "
                          f"(son: acc={msg.xacc:.2f},{msg.yacc:.2f},{msg.zacc:.2f})")

                now = time.time()
                if now - last_gps_sent > GPS_UPDATE_PERIOD_S:
                    last_gps_sent = now
                    px4.mav.hil_gps_send(
                        time_usec,
                        3,  # fix_type: 3D fix
                        int(FAKE_LAT_DEG * 1e7),
                        int(FAKE_LON_DEG * 1e7),
                        int(FAKE_ALT_M * 1000),
                        eph=100, epv=100,  # cm, deliberately poor accuracy
                        vel=0, vn=0, ve=0, vd=0,
                        cog=0,
                        satellites_visible=10,
                    )

            # drain anything PX4 sends back (e.g. HIL_ACTUATOR_CONTROLS) so the
            # socket buffer doesn't fill up; we don't act on it in this bridge.
            while True:
                reply = px4.recv_match(blocking=False)
                if reply is None:
                    break

    except KeyboardInterrupt:
        print(f"\n[*] Durduruldu. Toplam {hil_sensor_count} HIL_SENSOR PX4'e iletildi.")
        sys.exit(0)


if __name__ == "__main__":
    main()
