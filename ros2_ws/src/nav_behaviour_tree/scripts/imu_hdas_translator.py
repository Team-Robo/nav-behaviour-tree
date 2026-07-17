#!/usr/bin/env python3
"""Converts the real robot's hdas_msg/Imu contract on /hdas/imu_chassis into a standard
sensor_msgs/Imu on /imu, for robot_localization's ekf_node to consume (see
nav2_bringup_plan.md Part 7). This is the mirror image of robocup_nav's
imu_hdas_translator.py (sensor_msgs -> hdas_msg, for the sim's bridge output) -- this node
runs unmodified against sim or real hardware since both publish the same hdas_msg/Imu
contract on /hdas/imu_chassis."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from hdas_msg.msg import Imu as HdasImu


# hdas_msg/Imu carries no noise/covariance fields, so these are hand-picked placeholders
# (same status as swerve_odometry.py's odom covariance -- tune once real sensor noise is
# characterized). Diagonal only. Leaving these at the sensor_msgs/Imu default of all-zero
# tells ekf_node the measurement is known with *zero* uncertainty (per REP-103/ROS
# convention, only a leading -1 means "no estimate"), which combined with ekf.yaml's
# near-zero initial_estimate_covariance produces an ill-conditioned filter and NaN output.
_ORIENTATION_COVARIANCE = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02]
_ANGULAR_VELOCITY_COVARIANCE = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
_LINEAR_ACCELERATION_COVARIANCE = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]


def _euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ImuHdasTranslator(Node):

    def __init__(self):
        super().__init__('imu_hdas_translator')
        self.pub = self.create_publisher(Imu, '/imu', 10)
        self.create_subscription(
            HdasImu, '/hdas/imu_chassis', self._imu_cb, 10)

    def _imu_cb(self, msg):
        out = Imu()
        out.header = msg.header
        # The sim bridge stamps the raw IMU with frame_id 'r1/chassis_imu_link/chassis_imu',
        # which is NOT in the TF tree (only 'chassis_imu_link' is, via base_link ->
        # chassis_imu_link). robot_localization can't transform that frame into base_link, so
        # it silently drops every IMU measurement and the EKF runs on wheel odometry alone
        # (yaw then drifts ~8 deg over a 35 s run -- see localization_slam_tuning_plan.md).
        # The axis handling below already expresses the data in base_link's convention, so
        # relabel it base_link and the EKF fuses it with an identity transform. Do NOT use
        # 'chassis_imu_link' here: that frame carries the physical ~180 deg roll, which the
        # EKF would then re-apply, corrupting the already-corrected data.
        out.header.frame_id = 'base_link'
        # chassis_imu_link is mounted rolled ~180 deg about X relative to base_link
        # (r1_sensors.urdf.xacro's chassis_imu_joint), which reverses the sensor's Y and Z
        # axes. Verified against ground truth: raw msg.yaw is base_link's
        # true yaw negated (mean error 0.9 deg across the bag once negated, vs. tens of
        # degrees and growing before). two_d_mode in ekf.yaml discards roll/pitch, so only
        # the Z-axis (yaw, vyaw) and Y-axis (ay) corrections below actually reach the filter;
        # X is left as-is since that axis isn't flipped by the mount.
        x, y, z, w = _euler_to_quaternion(0.0, 0.0, -msg.yaw)
        out.orientation.x = x
        out.orientation.y = y
        out.orientation.z = z
        out.orientation.w = w
        out.angular_velocity.x = msg.groy_x
        out.angular_velocity.y = -msg.groy_y
        out.angular_velocity.z = -msg.groy_z
        out.linear_acceleration.x = msg.acc_x
        out.linear_acceleration.y = -msg.acc_y
        out.linear_acceleration.z = -msg.acc_z
        out.orientation_covariance = _ORIENTATION_COVARIANCE
        out.angular_velocity_covariance = _ANGULAR_VELOCITY_COVARIANCE
        out.linear_acceleration_covariance = _LINEAR_ACCELERATION_COVARIANCE
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuHdasTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
