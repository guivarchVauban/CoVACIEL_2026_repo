#!/bin/bash

# Configuration des variables d'environnement pour Webots
export WEBOTS_HOME=/usr/local/webots
export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$LD_LIBRARY_PATH
export PYTHONPATH=$WEBOTS_HOME/lib/controller/python:$PYTHONPATH
export ROS_DOMAIN_ID=42

# Source de ROS 2 Humble
source /opt/ros/humble/setup.bash

echo "--- 🛠️  Compilation du projet ---"
# On compile uniquement ton package pour gagner du temps
colcon build --packages-select nav_tunnel

if [ $? -eq 0 ]; then
    echo "--- ✅ Compilation réussie ! ---"
    # Source du workspace
    source install/setup.bash
    echo "--- 🚀 Environnement prêt. ROS_DOMAIN_ID=$ROS_DOMAIN_ID ---"
else
    echo "--- ❌ Erreur lors de la compilation ---"
fi
