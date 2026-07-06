#!/usr/bin/env python3
"""Independent Kalman-filtered attitude estimate, cross-checked against PX4's
own EKF2 output for anomaly detection.

Runs a classic 2-state (angle, gyro-bias) scalar Kalman filter on the raw
gyro + accelerometer readings from /fmu/out/sensor_combined - both ultimately
sourced from the real STM32 GY-87 hardware via the HITL pipeline - and
compares the result against PX4's EKF2 attitude estimate on
/fmu/out/vehicle_attitude. A sustained large disagreement between the two
independent estimators is flagged as a sensor/estimator anomaly.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleAttitude, SensorCombined

from uav_gcs_bridge.attitude_listener import quat_to_euler_deg

ANOMALY_THRESHOLD_DEG = 20.0


class AngleKalmanFilter:
    """Classic 2-state (angle, gyro-bias) scalar Kalman filter."""

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

        y = ((new_angle - self.angle + 180.0) % 360.0) - 180.0  # wrap innovation to [-180, 180)
        self.angle += k0 * y
        self.bias += k1 * y

        p00, p01 = P[0][0], P[0][1]
        P[0][0] -= k0 * p00
        P[0][1] -= k0 * p01
        P[1][0] -= k1 * p00
        P[1][1] -= k1 * p01

        return self.angle


class KalmanFusion(Node):

    def __init__(self):
        super().__init__('kalman_fusion')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.kf_roll = AngleKalmanFilter()
        self.kf_pitch = AngleKalmanFilter()
        self.last_sensor_ts_us = None

        self.ekf2_roll_deg = None
        self.ekf2_pitch_deg = None

        self.count = 0

        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self.on_sensor, qos_profile)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.on_attitude, qos_profile)

        self.get_logger().info('kalman_fusion started: kendi Kalman tahminimizi PX4 EKF2 ile karsilastiriyoruz...')

    def on_attitude(self, msg):
        self.ekf2_roll_deg, self.ekf2_pitch_deg, _ = quat_to_euler_deg(msg.q)

    def on_sensor(self, msg):
        import math

        if self.last_sensor_ts_us is None:
            self.last_sensor_ts_us = msg.timestamp
            return

        dt = (msg.timestamp - self.last_sensor_ts_us) * 1e-6
        self.last_sensor_ts_us = msg.timestamp
        if dt <= 0.0 or dt > 1.0:
            return

        ax, ay, az = msg.accelerometer_m_s2
        gx, gy, gz = msg.gyro_rad

        # NOTE: this HIL_SENSOR feed reads ~+g on Z at level attitude (not the
        # -g a strict FRD/NED convention would give), so the sign on Z (and
        # therefore on the X term too) is flipped relative to the textbook
        # formula - fitted empirically against PX4's own EKF2 output.
        roll_accel_deg = math.degrees(math.atan2(ay, -az))
        pitch_accel_deg = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))

        roll_kf = self.kf_roll.update(roll_accel_deg, math.degrees(gx), dt)
        pitch_kf = self.kf_pitch.update(pitch_accel_deg, math.degrees(gy), dt)

        self.count += 1
        if self.count % 20 != 0:
            return

        line = f'KF: roll={roll_kf:+7.2f} pitch={pitch_kf:+7.2f} (deg)'

        if self.ekf2_roll_deg is not None:
            d_roll = abs(((roll_kf - self.ekf2_roll_deg + 180.0) % 360.0) - 180.0)
            d_pitch = abs(((pitch_kf - self.ekf2_pitch_deg + 180.0) % 360.0) - 180.0)
            line += (f'  | EKF2: roll={self.ekf2_roll_deg:+7.2f} pitch={self.ekf2_pitch_deg:+7.2f}'
                     f'  | fark: droll={d_roll:5.1f} dpitch={d_pitch:5.1f}')

            if d_roll > ANOMALY_THRESHOLD_DEG or d_pitch > ANOMALY_THRESHOLD_DEG:
                self.get_logger().warn(f'ANOMALI: KF ve EKF2 tahminleri {ANOMALY_THRESHOLD_DEG} derece uzerinde ayrisiyor! {line}')
                return

        self.get_logger().info(line)


def main():
    rclpy.init()
    node = KalmanFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
