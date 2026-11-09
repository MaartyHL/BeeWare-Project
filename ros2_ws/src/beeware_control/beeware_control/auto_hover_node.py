#!/usr/bin/env python3
"""
BeeWare Project - Nœud de vol automatique : stationnaire (hover) + suivi ArUco, asservi en PID.

Principe :
  - Boucle externe : état du véhicule lu sur /fmu/out/* (attitude, vitesses, position)
    + erreurs ArUco reçues en UDP (127.0.0.1:9876) depuis beeware_vision.
  - Boucle interne (50 Hz par défaut) : PID -> /fmu/in/vehicle_rates_setpoint
    (mode body_rate : fonctionne sans GPS, validé en banc) + /fmu/in/offboard_control_mode.

Modes (topic /beeware/mode_command, std_msgs/String) :
  "auto"  : par défaut en Offboard — suit le marqueur ArUco ID 0 dès qu'il est visible
            (yaw selon err_x, trim de poussée selon err_y, avance/recul selon la taille),
            sinon stationnaire PID (altitude, assiette, cap) sur la position capturée.
            "track" est accepté comme alias de "auto".
  "hover" : stationnaire PID seul (ignore le marqueur)
  "off"   : stoppe la publication -> PX4 quitte l'Offboard, la RC reprend la main

Sécurité : bench_mode (plage de poussée réduite), deadband, lissage exponentiel,
clamps de rates/poussée, watchdog vision, anti-windup des PID.
Gains de départ à régler sur banc d'essai (voir config/auto_hover.yaml).
"""

import math
import os
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleRatesSetpoint,
    VehicleStatus,
    VehicleAttitude,
    VehicleAngularVelocity,
    VehicleLocalPosition,
)
from std_msgs.msg import String

try:
    from beeware_control.pid import PID
except ImportError:  # exécution directe : python3 .../beeware_control/auto_hover_node.py
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from beeware_control.pid import PID


def clamp(v, v_min, v_max):
    return max(v_min, min(v_max, v))


def wrap_angle(a):
    """Ramène un angle dans [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_to_euler(q):
    """Quaternion PX4 (w, x, y, z, repère NED) -> (roll, pitch, yaw) en rad."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class AutoHoverNode(Node):
    def __init__(self):
        super().__init__('auto_hover_node')

        # ---------- Paramètres ----------
        self.declare_parameter('bench_mode', True)
        self.declare_parameter('mode', 'auto')  # off | hover | auto
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('udp_port', 9876)

        # Poussée (repère NED : z négatif = vers le haut)
        self.declare_parameter('base_thrust', -0.20)
        self.declare_parameter('bench_thrust_min', -0.35)
        self.declare_parameter('bench_thrust_max', -0.05)
        self.declare_parameter('flight_base_thrust', -0.45)
        self.declare_parameter('flight_thrust_min', -0.80)
        self.declare_parameter('flight_thrust_max', -0.10)

        # Clamps et sécurités
        self.declare_parameter('max_rate_rp', 0.6)   # rad/s, roll/pitch
        self.declare_parameter('max_yaw_rate', 1.2)  # rad/s
        self.declare_parameter('watchdog_s', 0.2)
        self.declare_parameter('deadband_px', 15.0)
        self.declare_parameter('smooth_alpha', 0.35)

        # PID altitude : erreur (m) -> delta de poussée
        self.declare_parameter('alt_kp', 0.30)
        self.declare_parameter('alt_ki', 0.05)
        self.declare_parameter('alt_kd', 0.20)
        self.declare_parameter('alt_out_max', 0.25)
        # PID assiette : erreur (rad) -> rate (rad/s)
        self.declare_parameter('att_kp', 2.5)
        self.declare_parameter('att_ki', 0.2)
        self.declare_parameter('att_kd', 0.3)
        # PID cap : erreur yaw (rad) -> yaw rate (rad/s)
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('yaw_ki', 0.1)
        self.declare_parameter('yaw_kd', 0.05)
        # Suivi ArUco
        self.declare_parameter('visu_yaw_kp', 1.2)      # err_x normalisée -> yaw rate
        self.declare_parameter('visu_alt_kp', 0.15)     # err_y normalisée -> trim de poussée
        self.declare_parameter('visu_alt_max', 0.10)
        self.declare_parameter('side_target_px', 120.0)  # taille du marqueur à la distance visée
        self.declare_parameter('dist_kp', 0.30)          # écart de taille -> assiette d'avance (rad)
        self.declare_parameter('dist_pitch_max', 0.25)

        p = lambda name: self.get_parameter(name).value

        self.bench_mode = bool(p('bench_mode'))
        self.mode = str(p('mode'))
        control_hz = float(p('control_hz'))
        self.watchdog_s = float(p('watchdog_s'))
        self.deadband_px = float(p('deadband_px'))
        self.smooth_alpha = float(p('smooth_alpha'))
        self.max_rate_rp = float(p('max_rate_rp'))
        self.max_yaw_rate = float(p('max_yaw_rate'))
        self.visu_yaw_kp = float(p('visu_yaw_kp'))
        self.visu_alt_kp = float(p('visu_alt_kp'))
        self.visu_alt_max = float(p('visu_alt_max'))
        self.side_target_px = float(p('side_target_px'))
        self.dist_kp = float(p('dist_kp'))
        self.dist_pitch_max = float(p('dist_pitch_max'))

        if self.bench_mode:
            self.base_thrust = float(p('base_thrust'))
            self.thrust_min = float(p('bench_thrust_min'))
            self.thrust_max = float(p('bench_thrust_max'))
        else:
            self.base_thrust = float(p('flight_base_thrust'))
            self.thrust_min = float(p('flight_thrust_min'))
            self.thrust_max = float(p('flight_thrust_max'))

        # ---------- QoS micro-ROS (identique au nœud de banc validé) ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------- Publishers PX4 ----------
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.rates_pub = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos)

        # ---------- Subscribers état véhicule ----------
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.on_status, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_attitude, qos)
        self.create_subscription(VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity', self.on_angular_velocity, qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_local_position, qos)
        self.create_subscription(String, '/beeware/mode_command', self.on_mode_cmd, 10)

        # Publication de l'état (mode courant), latched pour supervision
        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mode_pub = self.create_publisher(String, '/beeware/mode', mode_qos)

        # ---------- UDP vision (protocole identique à aruco_live_stream.py) ----------
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('127.0.0.1', int(p('udp_port'))))
        self.udp_sock.setblocking(False)

        # ---------- État vision ----------
        self.err_x = 0.0
        self.err_y = 0.0
        self.side_len = 0.0
        self.last_seen = 0.0

        # ---------- État véhicule ----------
        self.arming_state = 0
        self.nav_state = -1
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate_meas = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.xy_valid = False
        self.z_valid = False
        self.v_valid = False
        self.have_state = False

        # ---------- Cibles de hover ----------
        self.x_t = 0.0
        self.y_t = 0.0
        self.z_t = 0.0
        self.yaw_t = 0.0
        self.targets_set = False

        # ---------- PID ----------
        alt_out_max = float(p('alt_out_max'))
        self.pid_alt = PID(p('alt_kp'), p('alt_ki'), p('alt_kd'),
                           -alt_out_max, alt_out_max, i_max=alt_out_max)
        self.pid_roll = PID(p('att_kp'), p('att_ki'), p('att_kd'),
                            -self.max_rate_rp, self.max_rate_rp, i_max=self.max_rate_rp)
        self.pid_pitch = PID(p('att_kp'), p('att_ki'), p('att_kd'),
                             -self.max_rate_rp, self.max_rate_rp, i_max=self.max_rate_rp)
        self.pid_yaw = PID(p('yaw_kp'), p('yaw_ki'), p('yaw_kd'),
                           -self.max_yaw_rate, self.max_yaw_rate, i_max=self.max_yaw_rate)

        # ---------- Sorties lissées ----------
        self.out_roll = 0.0
        self.out_pitch = 0.0
        self.out_yaw = 0.0
        self.out_thrust = self.base_thrust

        self.last_time = time.time()
        self._warned_no_z = False
        self._cycle = 0

        self.timer = self.create_timer(1.0 / control_hz, self.timer_callback)

        mode_str = "BANC (sans hélices)" if self.bench_mode else "VOL RÉEL"
        self.get_logger().info(
            f"auto_hover_node prêt : mode={self.mode}, {mode_str}, "
            f"base_thrust={self.base_thrust}, hz={control_hz}")
        self.publish_mode()

    # ---------- Callbacks d'état ----------

    def on_status(self, msg):
        self.arming_state = msg.arming_state
        self.nav_state = msg.nav_state
        self.have_state = True

    def on_attitude(self, msg):
        self.roll, self.pitch, self.yaw = quat_to_euler(msg.q)
        self.have_state = True

    def on_angular_velocity(self, msg):
        self.roll_rate = msg.xyz[0]
        self.pitch_rate = msg.xyz[1]
        self.yaw_rate_meas = msg.xyz[2]

    def on_local_position(self, msg):
        self.x, self.y, self.z = msg.x, msg.y, msg.z
        self.vx, self.vy, self.vz = msg.vx, msg.vy, msg.vz
        self.xy_valid = bool(msg.xy_valid)
        self.z_valid = bool(msg.z_valid)
        self.v_valid = bool(msg.v_xy_valid) and bool(msg.v_z_valid)
        self.have_state = True

    def on_mode_cmd(self, msg):
        m = msg.data.strip().lower()
        if m == 'track':
            m = 'auto'  # alias : suivi ArUco dès que la cible est visible, sinon hover
        if m not in ('off', 'hover', 'auto'):
            self.get_logger().warn(f"Mode inconnu ignoré : '{msg.data}' (attendu: off|hover|auto)")
            return
        if m == self.mode:
            return
        self.mode = m
        if m in ('hover', 'auto'):
            self.capture_targets()
            self.reset_pids()
        self.publish_mode()
        self.get_logger().info(f"Mode demandé -> {m.upper()}")

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    # ---------- Utilitaires ----------

    def capture_targets(self):
        """Mémorise la position/cap courants comme cibles de hover."""
        if not self.have_state:
            self.targets_set = False
            return
        self.x_t, self.y_t = self.x, self.y
        self.z_t = self.z if self.z_valid else 0.0
        self.yaw_t = self.yaw
        self.targets_set = True
        self.get_logger().info(
            f"Cibles hover capturées : x={self.x_t:.2f} y={self.y_t:.2f} "
            f"z={self.z_t:.2f} yaw={self.yaw_t:.2f}")

    def reset_pids(self):
        self.pid_alt.reset()
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()

    def read_vision(self):
        """Lit tous les paquets UDP en attente (format: "1,err_x,err_y,side_len")."""
        now = time.time()
        try:
            while True:
                data, _ = self.udp_sock.recvfrom(1024)
                parts = data.decode('utf-8').strip().split(',')
                if len(parts) >= 4 and parts[0] == '1':
                    self.last_seen = now
                    err_x = float(parts[1])
                    err_y = float(parts[2])
                    self.side_len = float(parts[3])
                    # Deadband : ignore les micro-bruits de pixels au centre
                    if abs(err_x) < self.deadband_px:
                        err_x = 0.0
                    if abs(err_y) < self.deadband_px:
                        err_y = 0.0
                    self.err_x = err_x
                    self.err_y = err_y
        except (BlockingIOError, socket.error):
            pass

    # ---------- Boucle de contrôle ----------

    def timer_callback(self):
        now = time.time()
        dt = clamp(now - self.last_time, 1e-3, 0.1)
        self.last_time = now

        self.read_vision()
        target_valid = (now - self.last_seen) < self.watchdog_s
        # En mode auto : on suit la cible dès qu'elle est visible, sinon on hover
        tracking = (self.mode == 'auto' and target_valid)

        # Capture des cibles dès que l'état véhicule est disponible
        if self.mode in ('hover', 'auto') and not self.targets_set and self.have_state:
            self.capture_targets()

        # ---------- Lois de commande ----------
        roll_rate_cmd = 0.0
        pitch_rate_cmd = 0.0
        yaw_rate_cmd = 0.0
        thrust_cmd = self.base_thrust

        if self.have_state:
            # Assiette visée : niveau, + avance/recul si track et cible valide
            pitch_sp = 0.0
            if tracking and self.side_target_px > 0.0 and self.side_len > 0.0:
                # marqueur plus petit que la cible = trop loin -> avancer (pitch négatif)
                err_dist = (self.side_target_px - self.side_len) / self.side_target_px
                pitch_sp = -clamp(self.dist_kp * err_dist, -self.dist_pitch_max, self.dist_pitch_max)

            # PID assiette -> rates (dérivée sur la mesure pour éviter le kick)
            roll_rate_cmd = self.pid_roll.compute(0.0 - self.roll, dt, derivative=-self.roll_rate)
            pitch_rate_cmd = self.pid_pitch.compute(pitch_sp - self.pitch, dt, derivative=-self.pitch_rate)

            if tracking:
                # Yaw visuel : marqueur à droite (err_x > 0) -> yaw positif (validé en banc)
                err_xn = clamp(self.err_x / 320.0, -1.0, 1.0)
                yaw_rate_cmd = clamp(self.visu_yaw_kp * err_xn,
                                     -self.max_yaw_rate, self.max_yaw_rate)
            else:
                # Hold de cap
                err_yaw = wrap_angle(self.yaw_t - self.yaw)
                yaw_rate_cmd = self.pid_yaw.compute(err_yaw, dt, derivative=-self.yaw_rate_meas)

            # PID altitude -> poussée (z NED négatif vers le haut)
            if self.z_valid:
                h = -self.z        # altitude, positive vers le haut
                h_t = -self.z_t
                err_h = h_t - h    # > 0 si trop bas
                d_meas = -self.vz if self.v_valid else None
                thrust_cmd -= self.pid_alt.compute(err_h, dt, derivative=d_meas)
            elif not self._warned_no_z:
                self.get_logger().warn(
                    "z non valide (pas d'estimation de hauteur) : hover en assiette seule, "
                    "poussée = base")
                self._warned_no_z = True

            # Suivi vertical ArUco : marqueur en bas de l'image (err_y > 0) -> descendre
            if tracking:
                err_yn = clamp(self.err_y / 240.0, -1.0, 1.0)
                thrust_cmd += clamp(self.visu_alt_kp * err_yn,
                                    -self.visu_alt_max, self.visu_alt_max)
        else:
            self._cycle += 1
            if self._cycle % 100 == 1:
                self.get_logger().warn("Aucun état véhicule reçu (/fmu/out/*) : setpoints neutres")

        # ---------- Clamps + lissage ----------
        roll_rate_cmd = clamp(roll_rate_cmd, -self.max_rate_rp, self.max_rate_rp)
        pitch_rate_cmd = clamp(pitch_rate_cmd, -self.max_rate_rp, self.max_rate_rp)
        yaw_rate_cmd = clamp(yaw_rate_cmd, -self.max_yaw_rate, self.max_yaw_rate)
        thrust_cmd = clamp(thrust_cmd, self.thrust_min, self.thrust_max)

        a = self.smooth_alpha
        self.out_roll = (1.0 - a) * self.out_roll + a * roll_rate_cmd
        self.out_pitch = (1.0 - a) * self.out_pitch + a * pitch_rate_cmd
        self.out_yaw = (1.0 - a) * self.out_yaw + a * yaw_rate_cmd
        self.out_thrust = (1.0 - a) * self.out_thrust + a * thrust_cmd

        # ---------- Log périodique ----------
        self._cycle += 1
        if self._cycle % 100 == 0:
            offboard = self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            self.get_logger().info(
                f"mode={self.mode} armé={armed} offboard={offboard} "
                f"rpy=({self.roll:.2f},{self.pitch:.2f},{self.yaw:.2f}) "
                f"z_valid={self.z_valid} suivi={'oui' if tracking else 'non'} "
                f"thrust={self.out_thrust:.2f}")

        # Mode "off" : on cesse de publier -> PX4 quitte l'Offboard, la RC reprend la main
        if self.mode == 'off':
            return

        # ---------- Publication PX4 (à ≥ 2 Hz, requis avant le switch Offboard) ----------
        ts = int(self.get_clock().now().nanoseconds / 1000)

        mode_msg = OffboardControlMode()
        mode_msg.timestamp = ts
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = True
        self.offboard_pub.publish(mode_msg)

        rates_msg = VehicleRatesSetpoint()
        rates_msg.timestamp = ts
        rates_msg.roll = self.out_roll
        rates_msg.pitch = self.out_pitch
        rates_msg.yaw = self.out_yaw
        rates_msg.thrust_body = [0.0, 0.0, self.out_thrust]
        self.rates_pub.publish(rates_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoHoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
