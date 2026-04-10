#!/bin/bash

# Fonction pour tout arrêter proprement quand tu feras Ctrl+C
function cleanup {
    echo "--- Arrêt de tous les nœuds ROS 2 ---"
    pkill -f static_transform_publisher
    pkill -f laser_scan_to_pc
    pkill -f imu_filter_madgwick_node
    pkill -f kiss_icp_node
    pkill -f ekf_node
    exit
}

trap cleanup SIGINT

echo "--- DÉMARRAGE DU SYSTÈME COVACIEL ---"

# 1. TFs Statiques
./script_tf.sh &
sleep 2

# 2. Lidar Scan -> PointCloud
ros2 run laser_scan_to_point_cloud laser_scan_to_pc --ros-args -p use_sim_time:=false &
sleep 2

# 3. Filtre IMU (Madgwick)
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p publish_tf:=false -p gain:=0.01 &
sleep 2

# 4. KISS-ICP (Odométrie Lidar)
ros2 run kiss_icp kiss_icp_node --ros-args --params-file /root/ros_ws/config/kiss_icp.yaml --remap /pointcloud_topic:=/scan_pc &
sleep 3

# 5. EKF (Fusion finale)
ros2 run robot_localization ekf_node --ros-args --params-file /root/ros_ws/config/ekf.yaml &

echo "✅ Tout est lancé ! Lance ton driver IMU à la main maintenant."
echo "Appuie sur Ctrl+C pour tout arrêter d'un coup."

wait
