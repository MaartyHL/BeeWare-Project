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
