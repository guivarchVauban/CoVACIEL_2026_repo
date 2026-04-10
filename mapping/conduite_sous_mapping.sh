#!/bin/bash

echo "🚀 Démarrage du mode Autonome IRL CoVACIEL (Mode Course)..."

# 1. Les TFs (le squelette)
./script_tf.sh &
PID_TF=$!
sleep 2

# 2. Le traducteur Lidar (Filtre de 30cm pour ignorer le châssis/câbles)
ros2 run laser_scan_to_point_cloud laser_scan_to_pc --ros-args \
    -p use_sim_time:=false \
    -p min_range:=0.30 &
PID_LSCAN=$!
sleep 2

# 3. L'odométrie KISS-ICP (Filtre de 30cm pour éviter de glisser sur le châssis)
ros2 run kiss_icp kiss_icp_node --ros-args \
    --params-file /root/ros_ws/config/kiss_icp.yaml \
    --remap /pointcloud_topic:=/scan_pc \
    -p min_range:=0.30 &
PID_KISS=$!
sleep 2

# 4. L'EKF (La fusion IMU/Lidar pour Nav2)
ros2 run robot_localization ekf_node --ros-args \
    --params-file /root/ros_ws/config/ekf.yaml &
PID_EKF=$!
sleep 2

# 5. Le Cerveau Nav2 (Correction Jazzy : suppression collision_monitor)
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=false \
    autostart:=true \
    map:=/root/ros_ws/maps/ma_belle_map.yaml \
    params_file:=/root/ros_ws/config/nav2_params.yaml \
    use_collision_monitor:=false &
PID_NAV2=$!

echo "✅ Système prêt ! Vitesse max réglée à 1.0 m/s."
echo "🌐 Sur ton PC (VM) : lance 'ros2 run rviz2 rviz2'"
echo "📍 N'oublie pas le '2D Pose Estimate' dans Rviz avant de donner un Goal !"
echo "🛑 Pour tout couper proprement, fais Ctrl+C."

# Capture du Ctrl+C pour arrêter tous les processus proprement
trap "echo 'Arrêt des processus...'; kill $PID_TF $PID_LSCAN $PID_KISS $PID_EKF $PID_NAV2; exit" INT TERM

wait
