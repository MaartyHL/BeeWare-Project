# Source le workspace BeeWare (sur la Pi, ros2_env est déjà activé par ~/.bashrc).
# Usage : source ~/BeeWare-Project/ros2_ws/source_beeware.sh
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$WS/install/setup.bash"

# colcon ne re-enregistre pas toujours les nouveaux packages ament_python dans
# install/setup.bash : on ajoute explicitement les préfixes des packages BeeWare.
for p in beeware_control beeware_vision beeware_bringup; do
    export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH:$WS/install/$p"
done
for p in beeware_control beeware_vision; do
    for d in "$WS"/install/$p/lib/python*/site-packages; do
        [ -d "$d" ] && export PYTHONPATH="$d:$PYTHONPATH"
    done
done
