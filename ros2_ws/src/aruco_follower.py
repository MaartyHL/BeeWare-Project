#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint
import socket

class ArucoFollowerNode(Node):
    def __init__(self):
        super().__init__('aruco_follower_node')
        
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

        self.rates_setpoint_pub = self.create_publisher(
            VehicleRatesSetpoint,
            '/fmu/in/vehicle_rates_setpoint',
            qos_profile
        )

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('127.0.0.1', 9876))
        self.udp_sock.setblocking(False)

        # Valeurs de vol stationnaire par défaut
        self.target_yaw_rate = 0.0
        self.target_thrust_z = -0.20

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Nœud de vol prêt : suivi ArUco 2D (Lacet + Altitude) actif !")

    def timer_callback(self):
        target_seen = False
        try:
            while True:
                data, _ = self.udp_sock.recvfrom(1024)
                data_str = data.decode('utf-8').strip()
                parts = data_str.split(',')
                
                if len(parts) >= 4 and parts[0] == '1':
                    target_seen = True
                    err_x = float(parts[1])
                    err_y = float(parts[2])
                    
                    # 1. Gestion du lacet (Yaw)
                    normalized_x = max(-1.0, min(1.0, err_x / 320.0))
                    self.target_yaw_rate = normalized_x * 1.5

                    # 2. Gestion de la hauteur (Thrust Z)
                    normalized_y = max(-1.0, min(1.0, err_y / 240.0))
                    raw_thrust = -0.20 + (normalized_y * 0.15)
                    self.target_thrust_z = max(-0.35, min(-0.05, raw_thrust))
        except (BlockingIOError, socket.error):
            pass

        # Si l'ArUco n'est plus détecté -> Vol stationnaire neutre
        if not target_seen:
            self.target_yaw_rate = 0.0
            self.target_thrust_z = -0.20

        # Envoi mode Offboard
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = True
        self.offboard_mode_pub.publish(mode_msg)

        # Envoi consignes
        rates_msg = VehicleRatesSetpoint()
        rates_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        rates_msg.roll = 0.0
        rates_msg.pitch = 0.0
        rates_msg.yaw = self.target_yaw_rate
        rates_msg.thrust_body = [0.0, 0.0, self.target_thrust_z]
        self.rates_setpoint_pub.publish(rates_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
