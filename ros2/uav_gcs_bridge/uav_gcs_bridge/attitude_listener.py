#!/usr/bin/env python3
"""Subscribes to PX4's uXRCE-DDS topics and prints attitude + raw sensor data.

This is the first ROS2-layer consumer of the real STM32 -> hil_bridge.py ->
PX4 SITL (HITL) pipeline: /fmu/out/vehicle_attitude and /fmu/out/sensor_combined
both ultimately trace back to real GY-87 hardware readings, not simulated
physics. Later nodes (Kalman fusion, waypoint planning) build on this same
subscription pattern.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleAttitude, SensorCombined


def quat_to_euler_deg(q):
    # q = [w, x, y, z], Hamilton convention, FRD body -> NED earth (PX4 uXRCE-DDS order)
    w, x, y, z = q

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class AttitudeListener(Node):

    def __init__(self):
        super().__init__('attitude_listener')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.latest_sensor = None
        self.count = 0

        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self.on_attitude, qos_profile)
        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self.on_sensor, qos_profile)

        self.get_logger().info('attitude_listener started, PX4 uXRCE-DDS topiclerini bekliyor...')

    def on_sensor(self, msg):
        self.latest_sensor = msg

    def on_attitude(self, msg):
        self.count += 1
        if self.count % 20 != 0:
            return

        roll, pitch, yaw = quat_to_euler_deg(msg.q)
        line = f'roll={roll:+7.2f} pitch={pitch:+7.2f} yaw={yaw:+7.2f} (deg)'

        if self.latest_sensor is not None:
            ax, ay, az = self.latest_sensor.accelerometer_m_s2
            gx, gy, gz = self.latest_sensor.gyro_rad
            line += f'  accel=({ax:+.2f},{ay:+.2f},{az:+.2f}) gyro=({gx:+.2f},{gy:+.2f},{gz:+.2f})'

        self.get_logger().info(line)


def main():
    rclpy.init()
    node = AttitudeListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
