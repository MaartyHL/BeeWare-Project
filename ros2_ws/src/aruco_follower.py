#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint
import socket
import time

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

        # ==========================================
        # SÉLECTEUR DE MODE D'EXÉCUTION
        # True  : Mode Banc d'essai (sans hélices, contrôle lacet + gaz audio bridés)
        # False : Mode Vol Réel (avec hélices, asservissement fermé)
        # ==========================================
        self.BENCH_MODE = True

        # Socket UDP
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('127.0.0.1', 9876))
        self.udp_sock.setblocking(False)

        # Paramètres de sécurité et de contrôle
        self.WATCHDOG_TIMEOUT = 0.20   # 200 ms max entre deux détections
        self.DEADBAND_PX = 15.0        # Zone morte de 15 px au centre
        self.SMOOTH_FACTOR = 0.35      # Lissage exponentiel (0 = lent, 1 = instantané)

        self.last_seen_time = 0.0
        self.current_yaw_rate = 0.0
        self.current_thrust_z = -0.20  # Poussée neutre par défaut

        self.timer = self.create_timer(0.05, self.timer_callback)
        mode_str = "BANC (Sans hélices)" if self.BENCH_MODE else "VOL RÉEL (Avec hélices)"
        self.get_logger().info(f"Nœud de vol initialisé en mode : {mode_str}")

    def timer_callback(self):
        now = time.time()
        raw_yaw_rate = 0.0
        raw_thrust_z = -0.20

        # 1. Lecture des paquets UDP de la caméra (format: "1,err_x,err_y,taille")
        try:
            while True:
                data, _ = self.udp_sock.recvfrom(1024)
                data_str = data.decode('utf-8').strip()
                parts = data_str.split(',')
                
                if len(parts) >= 4 and parts[0] == '1':
                    self.last_seen_time = now
                    err_x = float(parts[1])
                    err_y = float(parts[2])

                    # Zone morte (Deadband) : ignore les micro-bruits de pixels
                    if abs(err_x) < self.DEADBAND_PX:
                        err_x = 0.0
                    if abs(err_y) < self.DEADBAND_PX:
                        err_y = 0.0

                    # Calcul Lacet (Yaw) : normalisé sur la demi-largeur (320 px)
                    norm_x = max(-1.0, min(1.0, err_x / 320.0))
                    raw_yaw_rate = norm_x * 1.2

                    # Calcul Hauteur (Thrust Z)
                    norm_y = max(-1.0, min(1.0, err_y / 240.0))
                    
                    if self.BENCH_MODE:
                        # --- MODE BANC DE TEST ---
                        # Variation de poussée pour entendre le régime moteur (5% à 35%)
                        target_thrust = -0.20 + (norm_y * 0.15)
                        raw_thrust_z = max(-0.35, min(-0.05, target_thrust))
                    else:
                        # --- MODE VOL RÉEL ---
                        # En vol réel, la poussée stationnaire dépendra de la boucle fermée
                        target_thrust = -0.30 + (norm_y * 0.10)
                        raw_thrust_z = max(-0.45, min(-0.15, target_thrust))

        except (BlockingIOError, socket.error):
            pass

        # 2. Watchdog de sécurité (Perte de cible > 200 ms)
        target_valid = (now - self.last_seen_time) < self.WATCHDOG_TIMEOUT
        if not target_valid:
            raw_yaw_rate = 0.0
            raw_thrust_z = -0.20 if self.BENCH_MODE else -0.30

        # 3. Filtre de lissage exponentiel
        self.current_yaw_rate = (1.0 - self.SMOOTH_FACTOR) * self.current_yaw_rate + (self.SMOOTH_FACTOR * raw_yaw_rate)
        self.current_thrust_z = (1.0 - self.SMOOTH_FACTOR) * self.current_thrust_z + (self.SMOOTH_FACTOR * raw_thrust_z)

        # 4. Envoi du contrat de mode Offboard
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = True
        self.offboard_mode_pub.publish(mode_msg)

        # 5. Envoi des consignes angulaires et poussée
        rates_msg = VehicleRatesSetpoint()
        rates_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        rates_msg.roll = 0.0
        rates_msg.pitch = 0.0
        rates_msg.yaw = self.current_yaw_rate
        rates_msg.thrust_body = [0.0, 0.0, self.current_thrust_z]
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