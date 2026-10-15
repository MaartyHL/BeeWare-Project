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

        # Socket UDP
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('127.0.0.1', 9876))
        self.udp_sock.setblocking(False)

        # --- PARAMÈTRES DE CONTRÔLE ET SÉCURITÉ ---
        self.WATCHDOG_TIMEOUT = 0.20   # 200 ms max entre 2 détections
        self.DEADBAND_PX = 15.0        # Zone morte de 15 pixels au centre
        self.SMOOTH_FACTOR = 0.35      # Facteur de lissage (entre 0 et 1)

        self.last_seen_time = 0.0
        self.current_yaw_rate = 0.0
        self.current_thrust_z = -0.20  # Gaz stationnaire neutre (20%)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Nœud de vol v2 démarré : Watchdog + Deadband + Filtre actifs !")

    def timer_callback(self):
        now = time.time()
        raw_yaw_rate = 0.0
        raw_thrust_z = -0.20

        # 1. Lecture de tous les paquets UDP en attente (on ne garde que le plus récent)
        try:
            while True:
                data, _ = self.udp_sock.recvfrom(1024)
                data_str = data.decode('utf-8').strip()
                parts = data_str.split(',')
                
                if len(parts) >= 4 and parts[0] == '1':
                    self.last_seen_time = now
                    err_x = float(parts[1])
                    err_y = float(parts[2])

                    # --- A. ZONE MORTE (Deadband) ---
                    # Si l'erreur est inférieure à 15 pixels, on la considère comme nulle
                    if abs(err_x) < self.DEADBAND_PX:
                        err_x = 0.0
                    if abs(err_y) < self.DEADBAND_PX:
                        err_y = 0.0

                    # --- B. CALCUL DES COMMANDES BRUTES ---
                    # Lacet : normalisé entre -1.0 et +1.0, bridé à 1.2 rad/s max (plus doux)
                    norm_x = max(-1.0, min(1.0, err_x / 320.0))
                    raw_yaw_rate = norm_x * 1.2

                    # Poussée verticale : variation de +/- 15% autour de -0.20
                    norm_y = max(-1.0, min(1.0, err_y / 240.0))
                    target_thrust = -0.20 + (norm_y * 0.15)
                    raw_thrust_z = max(-0.35, min(-0.05, target_thrust))

        except (BlockingIOError, socket.error):
            pass

        # --- 2. SÉCURITÉ WATCHDOG (Perte de cible) ---
        # Si aucune cible valide reçue depuis plus de 200 ms -> vol stationnaire neutre
        target_valid = (now - self.last_seen_time) < self.WATCHDOG_TIMEOUT
        if not target_valid:
            raw_yaw_rate = 0.0
            raw_thrust_z = -0.20

        # --- 3. FILTRE DE LISSAGE (Évite les secousses brutales) ---
        # Formule : nouvelle_valeur = ancien*0.65 + nouveau*0.35
        self.current_yaw_rate = (1.0 - self.SMOOTH_FACTOR) * self.current_yaw_rate + (self.SMOOTH_FACTOR * raw_yaw_rate)
        self.current_thrust_z = (1.0 - self.SMOOTH_FACTOR) * self.current_thrust_z + (self.SMOOTH_FACTOR * raw_thrust_z)

        # Log d'info si la cible est active
        if target_valid and (abs(raw_yaw_rate) > 0.01 or abs(raw_thrust_z - (-0.20)) > 0.01):
            gaz_pct = abs(self.current_thrust_z) * 100
            self.get_logger().info(f"[TRACKING] Yaw: {self.current_yaw_rate:+.2f} rad/s | Gaz: {gaz_pct:.0f}%")

        # 4. Envoi du mode Offboard
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = True
        self.offboard_mode_pub.publish(mode_msg)

        # 5. Envoi des consignes lissées
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
