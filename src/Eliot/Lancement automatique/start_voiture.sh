#!/bin/bash
# On laisse 20 secondes pour que tout soit prêt (USB, Docker, etc.)
sleep 20

# 1. Lancer la vidéo Goofy
screen -dmS video python3 /home/util/flux_video_goofy/goofyvideo.py

# 2. S'assurer que le container est bien en route
sudo docker start covaciel_lidar
sleep 5

# 3. Lancer le node de contrôle ROS 2 (Vérifie bien que TOUTE la ligne est là)
screen -dmS ros sudo docker exec covaciel_lidar bash -c "source /root/ros_ws/install/setup.bash && ros2 run controlle>

echo "✅ Tout est lancé. Utilise 'screen -ls' pour vérifier."


