# 🐝 BeeWare Project

> **Drone autonome de détection et de suivi de frelons asiatiques par fusion audio-visuelle.**  
> Projet Robotique ROB4.

---

## 🚁 Spécifications Matérielles

| Composant | Modèle / Spécification | Rôle |
|---|---|---|
| **Châssis** | Quadcopter custom en impression 3D | Structure porteuse |
| **Contrôleur de vol** | Pixhawk 2.4.8 | Bas-niveau (stabilisation, EKF de vol, moteurs) |
| **Ordinateur de bord** | Raspberry Pi 5 | Haut-niveau (ROS 2, fusion de capteurs, navigation) |
| **Accélérateur IA** | NPU 13 TOPS (M.2 HAT+ / Hailo-8L) | Inférence temps réel sur modèles de détection |
| **Caméra** | Global Shutter (Avant) | Perception visuelle, détection d'obstacles / cibles rapides |
| **Propulsion** | 4 moteurs 2700 KV + ESC 4-en-1 70A | Motorisation |
| **Alimentation** | Power Module (Pixhawk/ESC) + Buck 5V (RPi 5) | Distribution électrique |
| **Communication** | Liaison série UART (Pixhawk TELEM <-> RPi 5 GPIO) | Télémétrie MAVLink / Micro-XRCE-DDS |

---

## 📂 Organisation du Répertoire

```text
BeeWare Project/
├── ros2_ws/             # Espace de travail ROS 2 embarqué
│   └── src/             # Packages ROS 2 (bringup, control, vision, audio)
├── ai/                  # Datasets, scripts d'entraînement et modèles compilés (.hef)
├── hardware/            # CAO 3D (STEP), impressions (.stl), schémas et paramètres de vol
├── design/              # Direction artistique, logos, visuels et photos du drone
├── Reports/             # Rapports d'avancement académiques ROB4
└── docs/                # Documentation technique et procédures d'installation
```

---

## 🛠️ Démarrage Rapide

### Connexion à la Raspberry Pi 5 en SSH
```bash
ssh martin@barrypi.local
```

### Développement à distance avec VS Code
Installez l'extension **Remote - SSH** (`ms-vscode-remote.remote-ssh`) dans VS Code :
1. `Ctrl+Shift+P` -> `Remote-SSH: Connect to Host...`
2. Entrez `martin@barrypi.local`
3. Ouvrez le dossier `/home/martin/BeeWare-Project/ros2_ws` sur la Pi.

### Build du workspace ROS 2 (sur la Pi, dans l'environnement ROS 2 Humble)
```bash
cd ~/BeeWare-Project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Lancer le vol automatique (banc d'essai d'abord, sans hélices)
Terminal 1 — vision (Python **système** 3.11, à cause de picamera2) :
```bash
python3 ~/BeeWare-Project/ros2_ws/src/beeware_vision/beeware_vision/aruco_live_stream.py
```
Terminal 2 — contrôle (environnement ROS 2) :
```bash
ros2 launch beeware_bringup auto_flight.launch.py bench_mode:=true mode:=auto
```
Ancien nœud de banc (servoing 2D yaw/poussée, validé) toujours disponible :
```bash
ros2 run beeware_control aruco_follower
```

### Comportement par défaut en Offboard (mode `auto`)
Dès que le marqueur ArUco ID 0 est visible, le drone le suit (yaw, poussée, avance selon la taille) ; sinon il fait du stationnaire PID. Aucun switch de mode nécessaire. Mode courant lisible sur `/beeware/mode`.

### Commandes de mode (topic `/beeware/mode_command`) — optionnelles
```bash
ros2 topic pub /beeware/mode_command std_msgs/msg/String "{data: 'hover'}" --once   # stationnaire PID seul (ignore le marqueur)
ros2 topic pub /beeware/mode_command std_msgs/msg/String "{data: 'auto'}" --once    # retour au suivi auto ('track' accepté comme alias)
ros2 topic pub /beeware/mode_command std_msgs/msg/String "{data: 'off'}" --once     # quitte l'Offboard, la RC reprend
```

### Surveiller le nœud
```bash
ros2 topic echo /fmu/in/vehicle_rates_setpoint
ros2 topic echo /fmu/out/vehicle_local_position
ros2 topic echo /fmu/out/vehicle_status
```

> Les gains PID de départ sont dans `ros2_ws/src/beeware_control/config/auto_hover.yaml` :
> à régler sur banc d'essai (sans hélices), puis en vol attaché, avant tout vol libre.
