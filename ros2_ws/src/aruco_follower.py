#!/usr/bin/env python3
"""
BeeWare Project - Nœud de Vol ROS 2 Offboard
Reçoit les détections ArUco depuis le nœud de vision (UDP 9876),
et commande les moteurs de la Pixhawk en mode OFFBOARD via Micro-XRCE-DDS.
"""

import socket
import select
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Messages officiels PX4
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint


class ArucoFlightNode(Node):
    def __init__(self):
        super().__init__('aruco_flight_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        # Réception UDP depuis le nœud de vision
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 9876))
        self.sock.setblocking(False)

        # Gains du régulateur (adaptés pour le test sur table)
        self.K_yaw = 0.003    # Vitesse de rotation (rad/s par pixel)
        self.K_z = 0.002      # Vitesse verticale (m/s)
        self.K_x = 0.3        # Vitesse avant/arrière (m/s)

        self.target_visible = False
        self.err_x = 0
        self.err_y = 0
        self.side_len = 0.0

        # Boucle de vol à 20 Hz (50 ms)
        self.timer = self.create_timer(0.05, self.flight_loop)
        self.get_logger().info("🚀 Nœud de vol ROS 2 prêt ! Prêt pour le mode Offboard.")

    def flight_loop(self):
        # 1. Lire les dernières coordonnées de vision disponibles
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                parts = data.decode('utf-8').split(',')
                if parts[0] == "1":
                    self.target_visible = True
                    self.err_x = int(parts[1])
                    self.err_y = int(parts[2])
                    self.side_len = float(parts[3])
                else:
                    self.target_visible = False
            except BlockingIOError:
                break

        # 2. Maintenir le mode OFFBOARD actif pour le watchdog PX4
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)

        # 3. Calcul des consignes de vitesse
        vx = 0.0
        vy = 0.0
        vz = 0.0
        yaw_rate = 0.0

        if self.target_visible:
            # Asservissement vers la cible
            yaw_rate = float(np.clip(self.K_yaw * self.err_x, -0.8, 0.8))
            vz = float(np.clip(self.K_z * self.err_y, -0.4, 0.4))
            
            target_side = 120.0
            err_dist = (target_side - self.side_len) / target_side
            vx = float(np.clip(self.K_x * err_dist, -0.3, 0.3))

            print(f"[VOL OFFBOARD] ArUco suivi -> Rotation: {yaw_rate:+.2f} rad/s | Vz: {vz:+.2f} m/s", end='\r')

        # 4. Envoi de la commande de vol à PX4
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity = [vx, vy, vz]
        setpoint.yawspeed = yaw_rate
        self.trajectory_setpoint_pub.publish(setpoint)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
