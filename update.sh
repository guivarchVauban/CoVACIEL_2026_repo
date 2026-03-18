#!/bin/bash

# 1. Définition des chemins (Basé sur tes logs précédents)
SOURCE_REPO="/home/util/CoVACIEL_2026_repo/src/Tom/ros2_ws/src/nav_tunnel/nav_tunnel/nav_tunnel.py"
DEST_ROS="/home/util/ros2_ws/src/nav_tunnel/nav_tunnel/nav_tunnel.py"

echo "--- 1. 🔄 Copie du fichier depuis le REPO vers ROS2_WS ---"
# On écrase l'ancien fichier avec ta nouvelle version
cp "$SOURCE_REPO" "$DEST_ROS"

if [ $? -eq 0 ]; then
    echo "✅ Copie réussie."
else
    echo "❌ Erreur : Fichier source introuvable !"
    exit 1
fi

echo "--- 2. 🔨 Compilation (Mise à jour) ---"
cd ~/ros2_ws
colcon build --packages-select nav_tunnel

echo "--- 3. 🚀 Lancement du robot ---"
source install/setup.bash
ros2 run nav_tunnel nav_tunnel
