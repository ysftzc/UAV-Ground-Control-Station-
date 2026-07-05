#!/usr/bin/env python3
"""HIL bridge: forward STM32 HIL_SENSOR (real IMU/baro/mag over UART) into
PX4 SITL's simulator_mavlink TCP link, so PX4's actual EKF2/attitude estimator
runs on real hardware sensor data instead of simulated physics.

PX4 side: run SITL in HITL mode first (no gz/jmavsim visual sim):
    cd ~/stm32_ws/PX4-Autopilot
    PX4_SIM_MODEL=none_iris make px4_sitl none_iris

That starts `simulator_mavlink` listening as a TCP server on 127.0.0.1:4560.

Rate note: the STM32 firmware only transmits HIL_SENSOR at ~10 Hz (it holds
the last I2C reading between its slower sensor task periods - see the comment
above MAVLink_TX_Task in Core/Src/freertos.c). PX4's HITL link expects a much
higher feed rate or it flags the baro/IMU as STALE and EKF2 refuses to start.
Rather than reflashing firmware to transmit faster, this bridge holds the
latest STM32 snapshot and *resends* it to PX4 on its own high-rate timer
(RESEND_RATE_HZ) - same "hold" semantics, just applied on the PC side where
it's cheap to iterate on.

Usage:
    .venv/bin/python tools/hil_bridge.py [--serial-port /dev/ttyUSB0] [--baud 115200]
                                          [--px4-host 127.0.0.1] [--px4-port 4560]
"""

import argparse
import random
import sys
import threading
import time

from pymavlink import mavutil

# No real GPS on the board yet: feed PX4 a fixed static fix so EKF2 can get a
# global position estimate. Eskisehir, Turkey, roughly at sea level.
FAKE_LAT_DEG = 39.7767
FAKE_LON_DEG = 30.5206
FAKE_ALT_M = 800.0
GPS_UPDATE_PERIOD_S = 0.2  # 5 Hz, well within what HIL_GPS needs

RESEND_RATE_HZ = 50  # PX4-facing rate; decoupled from the STM32's ~10 Hz native rate

# PX4's DataValidator flags a sensor STALE if it reports the exact same
# bit-identical value too many times in a row (a stuck-sensor failsafe - see
# DataValidator::set_equal_value_threshold). Since we resend the same cached
# STM32 snapshot between its real ~10 Hz updates, every field would otherwise
# be bit-identical across dozens of resends. Dither each resend by an amount
# well under the sensor's real noise floor to avoid false-positiving that
# check without fabricating meaningfully different readings.
JITTER = {
    "acc": 0.005,     # m/s^2
    "gyro": 0.0005,   # rad/s
    "mag": 0.001,     # gauss
    "pressure": 0.01,  # hPa
    "alt": 0.01,      # m
    "temp": 0.01,     # degC
}


def dither(value, scale):
    return value + random.uniform(-scale, scale)


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

    lock = threading.Lock()
    latest = {"msg": None, "stm32_count": 0}
    stop = threading.Event()
    t_start = time.time()

    def resend_loop():
        sent_count = 0
        last_gps_sent = 0.0
        period = 1.0 / RESEND_RATE_HZ

        while not stop.is_set():
            loop_start = time.time()

            with lock:
                msg = latest["msg"]

            if msg is not None:
                time_usec = int((time.time() - t_start) * 1e6)
                px4.mav.hil_sensor_send(
                    time_usec,
                    dither(msg.xacc, JITTER["acc"]), dither(msg.yacc, JITTER["acc"]),
                    dither(msg.zacc, JITTER["acc"]),
                    dither(msg.xgyro, JITTER["gyro"]), dither(msg.ygyro, JITTER["gyro"]),
                    dither(msg.zgyro, JITTER["gyro"]),
                    dither(msg.xmag, JITTER["mag"]), dither(msg.ymag, JITTER["mag"]),
                    dither(msg.zmag, JITTER["mag"]),
                    dither(msg.abs_pressure, JITTER["pressure"]), msg.diff_pressure,
                    dither(msg.pressure_alt, JITTER["alt"]),
                    dither(msg.temperature, JITTER["temp"]),
                    msg.fields_updated,
                    0,  # id: primary IMU, drives PX4 lockstep clock
                )
                sent_count += 1

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

                if sent_count % (RESEND_RATE_HZ * 5) == 0:
                    with lock:
                        stm32_count = latest["stm32_count"]
                    print(f"[*] PX4'e {sent_count} HIL_SENSOR gonderildi "
                          f"({RESEND_RATE_HZ}Hz resend, STM32'den {stm32_count} gercek ornek geldi)")

            # drain anything PX4 sends back (HIL_ACTUATOR_CONTROLS etc.)
            while True:
                reply = px4.recv_match(blocking=False)
                if reply is None:
                    break

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, period - elapsed))

    sender = threading.Thread(target=resend_loop, daemon=True)
    sender.start()

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
                with lock:
                    latest["msg"] = msg
                    latest["stm32_count"] += 1

    except KeyboardInterrupt:
        stop.set()
        with lock:
            stm32_count = latest["stm32_count"]
        print(f"\n[*] Durduruldu. STM32'den {stm32_count} ornek geldi.")
        sys.exit(0)


if __name__ == "__main__":
    main()
