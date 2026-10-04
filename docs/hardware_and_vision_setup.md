# 🛸 Configuration Matérielle & Pipeline de Vision

Ce document récapitule l'architecture validée et les procédures de mise en route de l'ordinateur de bord (Raspberry Pi 5) avec le contrôleur de vol (Pixhawk 2.4.8 PX4).

---

## 1. Schéma de Câblage UART (Pixhawk <-> Raspberry Pi 5)

La communication s'effectue via le port **TELEM 2** de la Pixhawk 2.4.8 et l'UART matériel du GPIO 40 broches de la Raspberry Pi 5 :

| Pixhawk TELEM 2 (Connecteur 6 broches) | Rôle | Broche Raspberry Pi 5 (Header 40 broches) |
|---|---|---|
| Broche 1 (+5V) | VCC | **NON CONNECTÉ** (Pi alimentée par son propre Buck 5V) |
| Broche 2 (TX Pixhawk) | Émission série | **Broche 10 (GPIO 15 - RX)** |
| Broche 3 (RX Pixhawk) | Réception série | **Broche 8 (GPIO 14 - TX)** |
| Broches 4 & 5 (CTS / RTS) | Contrôle de flux | **NON CONNECTÉS** |
| Broche 6 (GND) | Masse commune | **Broche 6 (GND)** |

> Vitesse de transmission configurée dans PX4 (`SER_TEL2_BAUD`) : **921 600 bauds**.

---

## 2. Périphériques Validés

1. **Accélérateur IA :** `Hailo Technologies Ltd. Hailo-8 AI Processor` (13 TOPS, Architecture HAILO8L, Firmware 4.23.0) sur bus PCIe Gen 3.
2. **Capteur Visuel :** `Sony IMX296` Global Shutter monochrome (1456x1088 jusqu'à 60 FPS) sur port caméra CSI RP1.
3. **Pont PX4 <-> ROS 2 :** `MicroXRCEAgent` actif en service systemd d'arrière-plan (`micro-xrce-agent.service`).

---

## 3. Détection de Marqueur ArUco & Retour Vidéo FPV

* **Dictionnaire :** `DICT_4X4_50`, Marqueur ID `0`.
* **Script de détection & streaming :** `ros2_ws/src/aruco_live_stream.py`.
* **Flux vidéo en direct :** Accessible sur `http://<IP_DRONE>:5000` via navigateur web.
