#!/usr/bin/env python3
"""
BeeWare Project - Nœud de Suivi Autonome ArUco (ROS 2 + PX4 Offboard)
Lit la caméra Global Shutter, calcule la position de l'ArUco,
et envoie les consignes de vitesse en direct à la Pixhawk via Micro-XRCE-DDS.
"""

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Messages officiels PX4
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

try:
    from picamera2 import Picamera2
except ImportError:
    print("Erreur: picamera2 n'est pas disponible.")
    exit(1)


class ArucoFollowerNode(Node):
    def __init__(self):
        super().__init__('aruco_follower')

        # Configuration QoS standard requise par PX4 DDS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers vers PX4
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

        # Initialisation de la caméra IMX296
        self.get_logger().info("Initialisation de la caméra Global Shutter...")
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)
        self.get_logger().info("Caméra prête !")

        # Configuration du détecteur ArUco
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            self.use_new_api = True
        else:
            self.use_new_api = False

        self.center_x = 640 // 2
        self.center_y = 480 // 2

        # Gains du régulateur proportionnel (ajustés doux pour les tests sur table)
        self.K_yaw = 0.003    # Vitesse de rotation (rad/s par pixel d'erreur)
        self.K_z = 0.002      # Vitesse verticale (m/s par pixel d'erreur)
        self.K_x = 0.3        # Vitesse avant/arrière (m/s)

        # Fréquence de la boucle de contrôle à 20 Hz (50 ms)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.last_seen_time = time.time()
        self.get_logger().info("Nœud ArUco Follower démarré à 20 Hz ! Prêt pour le mode Offboard.")

    def control_loop(self):
        # 1. Publication continue du mode OFFBOARD (requis par le watchdog PX4 à >= 2Hz)
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)

        # 2. Capture de l'image
        frame = self.picam2.capture_array()
        if len(frame.shape) == 2:
            gray = frame
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 3. Détection ArUco
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        vx = 0.0
        vy = 0.0
        vz = 0.0
        yaw_rate = 0.0

        now = time.time()

        if ids is not None and len(ids) > 0:
            self.last_seen_time = now
            c = corners[0][0]
            marker_center_x = int(np.mean(c[:, 0]))
            marker_center_y = int(np.mean(c[:, 1]))

            # Erreur par rapport au centre de l'image (en pixels)
            err_x = marker_center_x - self.center_x  # > 0 si cible à droite
            err_y = marker_center_y - self.center_y  # > 0 si cible en bas

            # Calcul de la taille apparente pour estimer la distance
            side_length = np.linalg.norm(c[0] - c[1])
            target_side = 120.0  # Taille cible en pixels correspondant à ~1 mètre
            err_dist = (target_side - side_length) / target_side

            # Consignes de vitesse :
            yaw_rate = np.clip(self.K_yaw * err_x, -0.6, 0.6)  # Rotation pour cadrer en X
            vz = np.clip(self.K_z * err_y, -0.3, 0.3)          # Descente/Montée pour cadrer en Y
            vx = np.clip(self.K_x * err_dist, -0.3, 0.3)       # Avancer/Reculer

            print(f"[TRACKING ID 0] ErrX: {err_x:+4d}px | ErrY: {err_y:+4d}px | Consignes -> YawRate: {yaw_rate:+.2f} rad/s, Vz: {vz:+.2f} m/s", end='\r')
        else:
            if now - self.last_seen_time > 1.0:
                # Si cible perdue : arrêt sur place (vol stationnaire)
                yaw_rate = 0.0
                vx = 0.0
                vy = 0.0
                vz = 0.0

        # 4. Envoi de la consigne de trajectoire à PX4
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity = [float(vx), float(vy), float(vz)]
        setpoint.yawspeed = float(yaw_rate)
        self.trajectory_setpoint_pub.publish(setpoint)

    def destroy_node(self):
        self.picam2.stop()
        super().destroy_node()


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
