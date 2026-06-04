#!/usr/bin/env python3
"""
CoVACIEL - Dashboard de monitoring ROS2
Subs tous les topics pertinents et sert un dashboard sur http://localhost:8080
Lancer DANS le container Docker : python3 dashboard.py
"""

import threading
import math
import time
import json
import subprocess
import psutil
from http.server import HTTPServer, BaseHTTPRequestHandler
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String, Float32, Int32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage
from rcl_interfaces.msg import Log


# ---------------------------------------------------------------------------
# État partagé (thread-safe via lock)
# ---------------------------------------------------------------------------
state_lock = threading.Lock()
state = {
    # --- Existant ---
    "robot_mode":       0,
    "course_active":    False,
    "bon_sens":         True,
    "sens_demi_tour":   "—",
    "obstacle_arriere": False,
    "ir_gauche_mm":     0.0,
    "ir_droit_mm":      0.0,
    "servo_angle":      90.0,
    "cmd_vel_linear":   0.0,
    "cmd_vel_angular":  0.0,
    "imu_yaw_deg":      0.0,
    "lidar_front_min":  0.0,
    "lidar_left_avg":   0.0,
    "lidar_right_avg":  0.0,
    # --- TF Tree ---
    "tf_transforms": {},      # dict: "parent->child" -> {age_ms, fresh}
    # --- Topic Health ---
    "topic_health": {
        "/scan":              {"hz": 0.0, "fresh": False, "last_ts": 0.0},
        "/odom":              {"hz": 0.0, "fresh": False, "last_ts": 0.0},
        "/imu/data":          {"hz": 0.0, "fresh": False, "last_ts": 0.0},
        "/cmd_vel":           {"hz": 0.0, "fresh": False, "last_ts": 0.0},
        "/odometry/filtered": {"hz": 0.0, "fresh": False, "last_ts": 0.0},
        "/cmd_vel_smoothed":  {"hz": 0.0, "fresh": False, "last_ts": 0.0},
    },
    # --- SLAM / Map ---
    "map_width":      0,
    "map_height":     0,
    "map_resolution": 0.0,
    "map_ts":         0.0,
    "map_occupied_pct": 0.0,
    # --- Logs rosout ---
    "rosout_logs": [],   # liste des 8 derniers logs [{level, name, msg, ts}]
    # --- Nodes status ---
    "nodes_status": {
        "/kiss_icp_node":        {"running": False, "last_seen": 0.0},
        "/ekf_filter_node":      {"running": False, "last_seen": 0.0},
        "/sllidar_node":         {"running": False, "last_seen": 0.0},
        "/orchestrateur_node":   {"running": False, "last_seen": 0.0},
        "/node_controller":      {"running": False, "last_seen": 0.0},
        "/controller_server":    {"running": False, "last_seen": 0.0},
        "/planner_server":       {"running": False, "last_seen": 0.0},
        "/amcl":                 {"running": False, "last_seen": 0.0},
        "/node_camera":          {"running": False, "last_seen": 0.0},
        "/node_xbee":            {"running": False, "last_seen": 0.0},
    },
    # --- AMCL pose ---
    "amcl_x":     0.0,
    "amcl_y":     0.0,
    "amcl_yaw":   0.0,
    "amcl_ts":    0.0,
    # --- System stats ---
    "cpu_pct":    0.0,
    "ram_pct":    0.0,
    "ram_used_mb": 0,
    "ram_total_mb": 0,
    "disk_pct":   0.0,
    "disk_used_gb": 0.0,
    "disk_total_gb": 0.0,
    "ts": 0.0,
}

MODE_LABELS = {0: "STOP", 1: "TUNNEL", 2: "U-TURN", 3: "SLAM"}

# Compteurs de fréquence par topic
_freq_counters = {
    "/scan":               deque(maxlen=30),
    "/odom":               deque(maxlen=30),
    "/imu/data":           deque(maxlen=30),
    "/cmd_vel":            deque(maxlen=30),
    "/odometry/filtered":  deque(maxlen=30),
    "/cmd_vel_smoothed":   deque(maxlen=30),
}


def _update_freq(topic: str):
    now = time.time()
    _freq_counters[topic].append(now)
    q = _freq_counters[topic]
    if len(q) >= 2:
        dt = q[-1] - q[0]
        hz = (len(q) - 1) / dt if dt > 0 else 0.0
    else:
        hz = 0.0
    with state_lock:
        state["topic_health"][topic]["hz"] = round(hz, 1)
        state["topic_health"][topic]["last_ts"] = now
        state["topic_health"][topic]["fresh"] = (now - state["topic_health"][topic]["last_ts"]) < 2.0


# ---------------------------------------------------------------------------
# Node ROS2
# ---------------------------------------------------------------------------
class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor_dashboard_node")

        # --- Topics existants ---
        self.create_subscription(Int32,   "/robot_mode",       self._cb_mode,      10)
        self.create_subscription(Bool,    "/course_active",    self._cb_course,    10)
        self.create_subscription(Bool,    "/bon_sens",         self._cb_bon_sens,  10)
        self.create_subscription(String,  "/sens_demi_tour",   self._cb_sens_dt,   10)
        self.create_subscription(Bool,    "/obstacle_arriere", self._cb_obs_arr,   10)
        self.create_subscription(Float32, "/ir_gauche",        self._cb_ir_g,      10)
        self.create_subscription(Float32, "/ir_droit",         self._cb_ir_d,      10)
        self.create_subscription(Float32, "/servo_angle",      self._cb_servo,     10)
        self.create_subscription(Twist,   "/cmd_vel",          self._cb_cmd,       10)
        self.create_subscription(Imu,     "/imu/data",         self._cb_imu,       10)
        self.create_subscription(LaserScan, "/scan",           self._cb_scan,      10)

        # --- Nouveaux ---
        self.create_subscription(TFMessage,     "/tf",         self._cb_tf,        10)
        self.create_subscription(TFMessage,     "/tf_static",  self._cb_tf_static, 10)
        self.create_subscription(OccupancyGrid, "/map",        self._cb_map,       10)
        self.create_subscription(Odometry,      "/odom",       self._cb_odom,      10)
        self.create_subscription(Log,           "/rosout",     self._cb_rosout,    50)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._cb_amcl, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._cb_odom_filtered, 10)
        self.create_subscription(Twist,    "/cmd_vel_smoothed",  self._cb_cmd_smoothed,  10)

        # Timer pour marquer les topics comme stale
        self.create_timer(1.0, self._check_freshness)
        # Timer pour checker les nodes actifs (ros2 node list)
        self.create_timer(3.0, self._check_nodes)
        # Timer pour les stats système
        self.create_timer(2.0, self._check_system)

        self.get_logger().info("Monitor dashboard node démarré")

    def _set(self, **kwargs):
        with state_lock:
            state.update(kwargs)
            state["ts"] = time.time()

    # --- Callbacks existants ---
    def _cb_mode(self, msg):       self._set(robot_mode=msg.data)
    def _cb_course(self, msg):     self._set(course_active=msg.data)
    def _cb_bon_sens(self, msg):   self._set(bon_sens=msg.data)
    def _cb_sens_dt(self, msg):    self._set(sens_demi_tour=msg.data)
    def _cb_obs_arr(self, msg):    self._set(obstacle_arriere=msg.data)
    def _cb_servo(self, msg):      self._set(servo_angle=msg.data)

    def _cb_ir_g(self, msg):
        raw = msg.data
        mm = 48000.0 / (raw - 20.0) if raw > 20 else 9999.0
        self._set(ir_gauche_mm=round(mm, 1))

    def _cb_ir_d(self, msg):
        raw = msg.data
        mm = 48000.0 / (raw - 20.0) if raw > 20 else 9999.0
        self._set(ir_droit_mm=round(mm, 1))

    def _cb_cmd(self, msg):
        _update_freq("/cmd_vel")
        self._set(
            cmd_vel_linear=round(msg.linear.x, 3),
            cmd_vel_angular=round(msg.angular.z, 3),
        )

    def _cb_imu(self, msg):
        _update_freq("/imu/data")
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny, cosy))
        self._set(imu_yaw_deg=round(yaw, 1))

    def _cb_scan(self, msg):
        _update_freq("/scan")
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        def avg_sector(start_deg, end_deg):
            start_i = int(start_deg / 360 * n) % n
            end_i   = int(end_deg   / 360 * n) % n
            if start_i <= end_i:
                sec = ranges[start_i:end_i]
            else:
                sec = ranges[start_i:] + ranges[:end_i]
            valid = [r for r in sec if msg.range_min < r < msg.range_max]
            return round(sum(valid) / len(valid), 3) if valid else 0.0

        front_sec = (
            [r for r in ranges[: int(25/360*n)] if msg.range_min < r < msg.range_max] +
            [r for r in ranges[int(335/360*n):] if msg.range_min < r < msg.range_max]
        )
        front_min = round(min(front_sec), 3) if front_sec else 0.0
        left_avg  = avg_sector(60,  120)
        right_avg = avg_sector(240, 300)

        self._set(
            lidar_front_min=front_min,
            lidar_left_avg=left_avg,
            lidar_right_avg=right_avg,
        )

    # --- Nouveaux callbacks ---
    def _cb_odom(self, msg):
        _update_freq("/odom")

    def _cb_tf(self, msg):
        now = time.time()
        tf_data = {}
        with state_lock:
            tf_data = dict(state["tf_transforms"])

        for t in msg.transforms:
            key = f"{t.header.frame_id}→{t.child_frame_id}"
            stamp_sec = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            age_ms = round((now - stamp_sec) * 1000, 0) if stamp_sec > 0 else -1
            tf_data[key] = {
                "age_ms": age_ms,
                "fresh": age_ms < 500 if age_ms >= 0 else True,
                "static": False,
            }

        with state_lock:
            state["tf_transforms"] = tf_data
            state["ts"] = now

    def _cb_tf_static(self, msg):
        now = time.time()
        with state_lock:
            for t in msg.transforms:
                key = f"{t.header.frame_id}→{t.child_frame_id}"
                state["tf_transforms"][key] = {
                    "age_ms": -1,
                    "fresh": True,
                    "static": True,
                }
            state["ts"] = now

    def _cb_map(self, msg):
        now = time.time()
        data = msg.data
        total = len(data)
        occupied = sum(1 for v in data if v > 50) if total > 0 else 0
        pct = round(occupied / total * 100, 1) if total > 0 else 0.0
        self._set(
            map_width=msg.info.width,
            map_height=msg.info.height,
            map_resolution=round(msg.info.resolution, 3),
            map_ts=now,
            map_occupied_pct=pct,
        )

    def _cb_rosout(self, msg):
        LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        level_str = LEVELS.get(msg.level, "?")
        if msg.level < 30:  # On ignore DEBUG et INFO pour pas spammer
            return
        entry = {
            "level": level_str,
            "name": msg.name,
            "msg": msg.msg[:120],
            "ts": time.time(),
        }
        with state_lock:
            logs = state["rosout_logs"]
            logs.append(entry)
            if len(logs) > 8:
                logs.pop(0)
            state["ts"] = time.time()

    def _check_freshness(self):
        now = time.time()
        with state_lock:
            for topic, health in state["topic_health"].items():
                last = health["last_ts"]
                health["fresh"] = (now - last) < 2.0 if last > 0 else False

    def _cb_odom_filtered(self, msg):
        _update_freq("/odometry/filtered")

    def _cb_cmd_smoothed(self, msg):
        _update_freq("/cmd_vel_smoothed")

    def _cb_amcl(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.degrees(math.atan2(siny, cosy))
        self._set(
            amcl_x=round(msg.pose.pose.position.x, 3),
            amcl_y=round(msg.pose.pose.position.y, 3),
            amcl_yaw=round(yaw, 1),
            amcl_ts=time.time(),
        )

    def _check_system(self):
        cpu = psutil.cpu_percent(interval=None)
        ram = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        with state_lock:
            state["cpu_pct"]      = round(cpu, 1)
            state["ram_pct"]      = round(ram.percent, 1)
            state["ram_used_mb"]  = ram.used // (1024*1024)
            state["ram_total_mb"] = ram.total // (1024*1024)
            state["disk_pct"]     = round(disk.percent, 1)
            state["disk_used_gb"] = round(disk.used / (1024**3), 1)
            state["disk_total_gb"]= round(disk.total / (1024**3), 1)
            state["ts"] = time.time()

    def _check_nodes(self):
        """Lance ros2 node list et met à jour nodes_status."""
        import subprocess, os
        try:
            result = subprocess.run(
                ["bash", "-c",
                 "source /opt/ros/jazzy/setup.bash && "
                 "export ROS_DOMAIN_ID=67 && "
                 "ros2 node list 2>/dev/null"],
                capture_output=True, text=True, timeout=8
            )
            active = set(result.stdout.strip().splitlines())
            now = time.time()
            with state_lock:
                for node_name in state["nodes_status"]:
                    running = node_name in active
                    state["nodes_status"][node_name]["running"] = running
                    if running:
                        state["nodes_status"][node_name]["last_seen"] = now
        except Exception as e:
            self.get_logger().warn(f"node list failed: {e}")


# ---------------------------------------------------------------------------
# HTML du dashboard
# ---------------------------------------------------------------------------
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CoVACIEL Monitor</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700;900&display=swap" rel="stylesheet">
<style>
  :root {
    --bg:       #080c10;
    --panel:    #0d1520;
    --border:   #1a2d45;
    --accent:   #00d4ff;
    --green:    #00ff88;
    --red:      #ff2244;
    --orange:   #ff8800;
    --yellow:   #ffe040;
    --purple:   #b060ff;
    --dim:      #3a5070;
    --text:     #c8e0f0;
    --mono:     'Share Tech Mono', monospace;
    --title:    'Orbitron', sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--mono);
    font-size: 13px;
    min-height: 100vh;
    padding: 16px;
  }
  body::before {
    content: '';
    position: fixed; inset: 0;
    background: repeating-linear-gradient(
      0deg, transparent, transparent 2px,
      rgba(0,0,0,.15) 2px, rgba(0,0,0,.15) 4px
    );
    pointer-events: none;
    z-index: 999;
  }

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 20px;
    border-bottom: 1px solid var(--border);
    padding-bottom: 12px;
  }
  header h1 {
    font-family: var(--title);
    font-size: 22px;
    font-weight: 900;
    letter-spacing: 4px;
    color: var(--accent);
    text-shadow: 0 0 20px var(--accent);
  }
  header h1 span { color: var(--green); }
  #conn-dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: var(--dim);
    transition: background .3s, box-shadow .3s;
  }
  #conn-dot.live {
    background: var(--green);
    box-shadow: 0 0 10px var(--green);
    animation: pulse 1.5s ease-in-out infinite;
  }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.4} }

  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(260px, 1fr));
    gap: 12px;
  }
  .panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 14px 16px;
    position: relative;
    overflow: hidden;
  }
  .panel::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: .5;
  }
  .panel.wide  { grid-column: span 2; }
  .panel.full  { grid-column: 1 / -1; }
  .panel-title {
    font-family: var(--title);
    font-size: 9px;
    letter-spacing: 3px;
    color: var(--dim);
    text-transform: uppercase;
    margin-bottom: 10px;
  }

  /* Mode badge */
  .mode-badge {
    font-family: var(--title);
    font-size: 32px;
    font-weight: 900;
    letter-spacing: 2px;
    text-align: center;
    padding: 10px 0;
    transition: color .3s, text-shadow .3s;
  }
  .mode-badge.m0 { color: var(--dim); }
  .mode-badge.m1 { color: var(--green);  text-shadow: 0 0 20px var(--green); }
  .mode-badge.m2 { color: var(--orange); text-shadow: 0 0 20px var(--orange); }
  .mode-badge.m3 { color: var(--accent); text-shadow: 0 0 20px var(--accent); }
  .mode-sub { text-align: center; color: var(--dim); font-size: 11px; margin-top: 4px; }

  /* Pills */
  .pills { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 4px; }
  .pill {
    padding: 5px 12px;
    border-radius: 2px;
    border: 1px solid;
    font-size: 11px;
    letter-spacing: 1px;
    transition: all .3s;
  }
  .pill.ok   { border-color: var(--green);  color: var(--green);  background: rgba(0,255,136,.08); }
  .pill.warn { border-color: var(--red);    color: var(--red);    background: rgba(255,34,68,.1);  box-shadow: 0 0 8px rgba(255,34,68,.3); }
  .pill.off  { border-color: var(--dim);    color: var(--dim);    background: transparent; }

  /* Rows */
  .row {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    padding: 5px 0;
    border-bottom: 1px solid rgba(26,45,69,.5);
  }
  .row:last-child { border-bottom: none; }
  .row .label { color: var(--dim); font-size: 11px; }
  .row .val   { font-size: 16px; color: var(--text); transition: color .2s; }
  .row .val.hi { color: var(--yellow); }
  .row .unit  { font-size: 10px; color: var(--dim); margin-left: 3px; }

  /* Speed bar */
  .bar-wrap { height: 8px; background: rgba(255,255,255,.05); border-radius: 2px; margin-top: 6px; overflow: hidden; }
  .bar-inner { height: 100%; border-radius: 2px; transition: width .3s, background .3s; }

  /* Servo arc */
  #servo-arc { display: block; margin: 8px auto 0; }

  /* ---- TF TREE ---- */
  .tf-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(200px, 1fr));
    gap: 6px;
    margin-top: 4px;
  }
  .tf-item {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 5px 8px;
    border-radius: 3px;
    border: 1px solid var(--border);
    background: rgba(0,0,0,.2);
    transition: border-color .3s;
  }
  .tf-item.fresh  { border-color: rgba(0,255,136,.3); }
  .tf-item.stale  { border-color: rgba(255,34,68,.5); background: rgba(255,34,68,.05); }
  .tf-item.static { border-color: rgba(176,96,255,.3); }
  .tf-name { font-size: 10px; color: var(--text); }
  .tf-age  { font-size: 10px; }
  .tf-age.fresh  { color: var(--green); }
  .tf-age.stale  { color: var(--red); }
  .tf-age.static { color: var(--purple); }

  /* ---- TOPIC HEALTH ---- */
  .health-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(170px, 1fr));
    gap: 8px;
    margin-top: 4px;
  }
  .health-card {
    padding: 10px 12px;
    border-radius: 3px;
    border: 1px solid var(--border);
    background: rgba(0,0,0,.2);
    transition: border-color .3s, background .3s;
  }
  .health-card.ok   { border-color: rgba(0,255,136,.4); background: rgba(0,255,136,.04); }
  .health-card.dead { border-color: rgba(255,34,68,.5); background: rgba(255,34,68,.06); animation: deadblink 1s ease-in-out infinite; }
  @keyframes deadblink { 0%,100%{opacity:1} 50%{opacity:.6} }
  .health-topic { font-size: 10px; color: var(--dim); margin-bottom: 4px; letter-spacing: 1px; }
  .health-hz    { font-family: var(--title); font-size: 22px; font-weight: 700; transition: color .3s; }
  .health-hz.ok   { color: var(--green); text-shadow: 0 0 10px rgba(0,255,136,.4); }
  .health-hz.dead { color: var(--red); }
  .health-unit  { font-size: 10px; color: var(--dim); margin-left: 3px; }
  .health-dot {
    display: inline-block;
    width: 6px; height: 6px;
    border-radius: 50%;
    margin-right: 6px;
    vertical-align: middle;
  }
  .health-dot.ok   { background: var(--green); box-shadow: 0 0 6px var(--green); }
  .health-dot.dead { background: var(--red); }

  /* ---- MAP STATUS ---- */
  .map-info { display: flex; gap: 16px; flex-wrap: wrap; margin-top: 6px; }
  .map-stat { text-align: center; flex: 1; min-width: 80px; }
  .map-stat .big {
    font-family: var(--title);
    font-size: 20px;
    font-weight: 700;
    color: var(--accent);
    text-shadow: 0 0 12px var(--accent);
  }
  .map-stat .lbl { font-size: 10px; color: var(--dim); margin-top: 2px; letter-spacing: 1px; }
  #map-status-dot {
    display: inline-block;
    width: 8px; height: 8px;
    border-radius: 50%;
    background: var(--dim);
    margin-right: 6px;
    vertical-align: middle;
  }
  #map-status-dot.active {
    background: var(--accent);
    box-shadow: 0 0 8px var(--accent);
    animation: pulse 2s ease-in-out infinite;
  }

  /* ---- ROSOUT LOGS ---- */
  .log-list { margin-top: 6px; display: flex; flex-direction: column; gap: 4px; max-height: 200px; overflow-y: auto; }
  .log-entry {
    display: flex;
    gap: 8px;
    padding: 4px 8px;
    border-radius: 2px;
    border-left: 2px solid var(--dim);
    background: rgba(0,0,0,.2);
    font-size: 11px;
    align-items: flex-start;
  }
  .log-entry.WARN  { border-left-color: var(--orange); background: rgba(255,136,0,.06); }
  .log-entry.ERROR { border-left-color: var(--red);    background: rgba(255,34,68,.08); }
  .log-entry.FATAL { border-left-color: var(--red);    background: rgba(255,34,68,.15); animation: deadblink .5s ease-in-out infinite; }
  .log-level { font-weight: bold; min-width: 38px; }
  .log-level.WARN  { color: var(--orange); }
  .log-level.ERROR { color: var(--red); }
  .log-level.FATAL { color: var(--red); }
  .log-name  { color: var(--dim); min-width: 120px; max-width: 120px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .log-msg   { color: var(--text); flex: 1; }
  .log-empty { color: var(--dim); font-size: 11px; font-style: italic; text-align: center; padding: 12px; }

  /* ---- NODES STATUS ---- */
  .nodes-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(160px, 1fr));
    gap: 6px;
    margin-top: 4px;
  }
  .node-card {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 6px 10px;
    border-radius: 3px;
    border: 1px solid var(--border);
    background: rgba(0,0,0,.2);
    transition: border-color .3s, background .3s;
  }
  .node-card.up   { border-color: rgba(0,255,136,.35); background: rgba(0,255,136,.04); }
  .node-card.down { border-color: rgba(255,34,68,.45); background: rgba(255,34,68,.06); animation: deadblink 1.2s ease-in-out infinite; }
  .node-dot {
    width: 7px; height: 7px;
    border-radius: 50%;
    flex-shrink: 0;
  }
  .node-dot.up   { background: var(--green); box-shadow: 0 0 6px var(--green); }
  .node-dot.down { background: var(--red); }
  .node-label { font-size: 10px; color: var(--text); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }

  /* ---- AMCL POSE ---- */
  .pose-grid { display: flex; gap: 12px; flex-wrap: wrap; margin-top: 6px; }
  .pose-stat { flex: 1; min-width: 70px; text-align: center; }
  .pose-stat .big {
    font-family: var(--title);
    font-size: 18px;
    font-weight: 700;
    color: var(--yellow);
    text-shadow: 0 0 10px rgba(255,224,64,.4);
  }
  .pose-stat .lbl { font-size: 10px; color: var(--dim); margin-top: 2px; letter-spacing: 1px; }

  footer {
    text-align: right;
    color: var(--dim);
    font-size: 10px;
    margin-top: 14px;
    letter-spacing: 1px;
  }
</style>
</head>
<body>

<header>
  <h1>CoVA<span>CIEL</span> / MONITOR</h1>
  <div style="display:flex;align-items:center;gap:8px;">
    <span id="conn-label" style="font-size:10px;color:var(--dim)">OFFLINE</span>
    <div id="conn-dot"></div>
  </div>
</header>

<div class="grid">

  <!-- Mode -->
  <div class="panel">
    <div class="panel-title">Mode Robot</div>
    <div class="mode-badge m0" id="mode-badge">STOP</div>
    <div class="mode-sub" id="mode-sub">En attente</div>
  </div>

  <!-- Flags -->
  <div class="panel">
    <div class="panel-title">Flags système</div>
    <div class="pills" id="pills-flags">
      <div class="pill off" id="pill-course">COURSE INACTIVE</div>
      <div class="pill ok"  id="pill-sens">BON SENS</div>
      <div class="pill off" id="pill-obs">OBS ARRIÈRE</div>
    </div>
    <div style="margin-top:12px;">
      <div class="panel-title">Demi-tour</div>
      <div id="sens-dt" style="font-size:14px;color:var(--accent);margin-top:4px;">—</div>
    </div>
  </div>

  <!-- IR -->
  <div class="panel">
    <div class="panel-title">Capteurs IR</div>
    <div class="row">
      <span class="label">◄ Gauche</span>
      <span><span class="val" id="ir-g">0</span><span class="unit">mm</span></span>
    </div>
    <div class="row">
      <span class="label">Droite ►</span>
      <span><span class="val" id="ir-d">0</span><span class="unit">mm</span></span>
    </div>
  </div>

  <!-- cmd_vel -->
  <div class="panel">
    <div class="panel-title">cmd_vel</div>
    <div class="row">
      <span class="label">Vitesse linéaire</span>
      <span><span class="val" id="vel-lin">0.000</span><span class="unit">m/s</span></span>
    </div>
    <div class="bar-wrap">
      <div class="bar-inner" id="vel-bar" style="width:0%;background:var(--green)"></div>
    </div>
    <div class="row" style="margin-top:8px;">
      <span class="label">Vitesse angulaire</span>
      <span><span class="val" id="vel-ang">0.000</span><span class="unit">rad/s</span></span>
    </div>
  </div>

  <!-- Servo -->
  <div class="panel">
    <div class="panel-title">Servo caméra</div>
    <svg id="servo-arc" width="160" height="90" viewBox="0 0 160 90">
      <path d="M 10 80 A 70 70 0 0 1 150 80" fill="none" stroke="rgba(26,45,69,.8)" stroke-width="6"/>
      <path id="servo-arc-fill" d="M 10 80 A 70 70 0 0 1 150 80" fill="none"
            stroke="var(--accent)" stroke-width="6" stroke-dasharray="220" stroke-dashoffset="220"
            style="transition:stroke-dashoffset .3s"/>
      <line id="servo-needle" x1="80" y1="80" x2="80" y2="18"
            stroke="var(--yellow)" stroke-width="2"
            style="transition:transform .3s;transform-origin:80px 80px;"/>
      <text x="80" y="76" text-anchor="middle" fill="var(--text)" font-family="Share Tech Mono"
            font-size="13" id="servo-text">90°</text>
    </svg>
  </div>

  <!-- IMU -->
  <div class="panel">
    <div class="panel-title">IMU — Cap (yaw)</div>
    <div class="row">
      <span class="label">Yaw</span>
      <span><span class="val" id="imu-yaw">0.0</span><span class="unit">°</span></span>
    </div>
    <svg width="80" height="80" viewBox="0 0 80 80" style="display:block;margin:8px auto 0;">
      <circle cx="40" cy="40" r="36" fill="none" stroke="var(--border)" stroke-width="1"/>
      <text x="40" y="12" text-anchor="middle" fill="var(--dim)" font-size="9" font-family="Share Tech Mono">N</text>
      <text x="40" y="74" text-anchor="middle" fill="var(--dim)" font-size="9" font-family="Share Tech Mono">S</text>
      <text x="8"  y="44" text-anchor="middle" fill="var(--dim)" font-size="9" font-family="Share Tech Mono">W</text>
      <text x="73" y="44" text-anchor="middle" fill="var(--dim)" font-size="9" font-family="Share Tech Mono">E</text>
      <line id="compass-needle" x1="40" y1="40" x2="40" y2="10"
            stroke="var(--red)" stroke-width="2"
            style="transition:transform .3s;transform-origin:40px 40px;"/>
      <circle cx="40" cy="40" r="3" fill="var(--accent)"/>
    </svg>
  </div>

  <!-- LiDAR -->
  <div class="panel wide">
    <div class="panel-title">LiDAR — distances clés</div>
    <div class="row">
      <span class="label">⬆ Frontal (min ±25°)</span>
      <span><span class="val" id="lid-front">0.000</span><span class="unit">m</span></span>
    </div>
    <div class="row">
      <span class="label">◄ Gauche (60–120°)</span>
      <span><span class="val" id="lid-left">0.000</span><span class="unit">m</span></span>
    </div>
    <div class="row">
      <span class="label">Droite (240–300°) ►</span>
      <span><span class="val" id="lid-right">0.000</span><span class="unit">m</span></span>
    </div>
  </div>

  <!-- ══════════════════════════════════════════════════ -->
  <!-- NOUVEAU : Topic Health                            -->
  <!-- ══════════════════════════════════════════════════ -->
  <div class="panel wide">
    <div class="panel-title">Topic Health — fréquences temps réel</div>
    <div class="health-grid" id="health-grid">
      <!-- injecté par JS -->
    </div>
  </div>

  <!-- ══════════════════════════════════════════════════ -->
  <!-- NOUVEAU : TF Tree                                 -->
  <!-- ══════════════════════════════════════════════════ -->
  <div class="panel full">
    <div class="panel-title">
      TF Tree —
      <span style="color:var(--green);font-size:9px;">● FRESH &lt;500ms</span>
      &nbsp;
      <span style="color:var(--red);font-size:9px;">● STALE</span>
      &nbsp;
      <span style="color:var(--purple);font-size:9px;">● STATIC</span>
    </div>
    <div class="tf-grid" id="tf-grid">
      <span style="color:var(--dim);font-size:11px;font-style:italic;">En attente des TFs...</span>
    </div>
  </div>

  <!-- ══════════════════════════════════════════════════ -->
  <!-- NOUVEAU : System Stats                            -->
  <!-- ══════════════════════════════════════════════════ -->
  <div class="panel wide">
    <div class="panel-title">Système — CPU / RAM / Stockage</div>
    <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;margin-top:4px;">

      <!-- CPU -->
      <div>
        <div style="font-size:9px;color:var(--dim);letter-spacing:2px;margin-bottom:6px;">CPU</div>
        <div style="font-family:var(--title);font-size:28px;font-weight:700;" id="cpu-pct">0.0</div>
        <div style="font-size:10px;color:var(--dim);">%</div>
        <div class="bar-wrap" style="margin-top:6px;">
          <div class="bar-inner" id="cpu-bar" style="width:0%;background:var(--green)"></div>
        </div>
      </div>

      <!-- RAM -->
      <div>
        <div style="font-size:9px;color:var(--dim);letter-spacing:2px;margin-bottom:6px;">RAM</div>
        <div style="font-family:var(--title);font-size:28px;font-weight:700;" id="ram-pct">0.0</div>
        <div style="font-size:10px;color:var(--dim);">% &nbsp;<span id="ram-detail" style="font-size:9px;"></span></div>
        <div class="bar-wrap" style="margin-top:6px;">
          <div class="bar-inner" id="ram-bar" style="width:0%;background:var(--accent)"></div>
        </div>
      </div>

      <!-- Disque -->
      <div>
        <div style="font-size:9px;color:var(--dim);letter-spacing:2px;margin-bottom:6px;">CARTE SD</div>
        <div style="font-family:var(--title);font-size:28px;font-weight:700;" id="disk-pct">0.0</div>
        <div style="font-size:10px;color:var(--dim);">% &nbsp;<span id="disk-detail" style="font-size:9px;"></span></div>
        <div class="bar-wrap" style="margin-top:6px;">
          <div class="bar-inner" id="disk-bar" style="width:0%;background:var(--purple)"></div>
        </div>
      </div>

    </div>
  </div>

  <!-- ══════════════════════════════════════════════════ -->
  <!-- NOUVEAU : Logs /rosout                            -->
  <!-- ══════════════════════════════════════════════════ -->
  <div class="panel wide">
    <div class="panel-title">Logs /rosout — WARN / ERROR / FATAL</div>
    <div class="log-list" id="log-list">
      <div class="log-empty">Aucun log WARN/ERROR pour l'instant</div>
    </div>
  </div>

  <!-- ══════════════════════════════════════════════════ -->
  <!-- NOUVEAU : Nodes Status                            -->
  <!-- ══════════════════════════════════════════════════ -->
  <div class="panel wide">
    <div class="panel-title">Nodes ROS2 — état en temps réel</div>
    <div class="nodes-grid" id="nodes-grid">
      <!-- injecté par JS -->
    </div>
    <div style="margin-top:8px;font-size:9px;color:var(--dim);">Rafraîchi toutes les 3s via ros2 node list</div>
  </div>

  <!-- ══════════════════════════════════════════════════ -->
  <!-- NOUVEAU : AMCL Pose                               -->
  <!-- ══════════════════════════════════════════════════ -->
  <div class="panel wide">
    <div class="panel-title">Position robot — AMCL Pose</div>
    <div class="pose-grid">
      <div class="pose-stat">
        <div class="big" id="amcl-x">—</div>
        <div class="lbl">X (m)</div>
      </div>
      <div class="pose-stat">
        <div class="big" id="amcl-y">—</div>
        <div class="lbl">Y (m)</div>
      </div>
      <div class="pose-stat">
        <div class="big" id="amcl-yaw">—</div>
        <div class="lbl">Cap (°)</div>
      </div>
    </div>
    <div style="margin-top:8px;font-size:10px;color:var(--dim);">
      Dernière pose : <span id="amcl-ts">Aucune pose reçue</span>
    </div>
  </div>

</div>

<footer id="ts-label">Dernière maj : —</footer>

<script>
const MODE_LABELS = {0:"STOP", 1:"TUNNEL", 2:"U-TURN", 3:"SLAM"};
const MODE_SUBS   = {0:"En attente", 1:"Wall-following PID", 2:"Manœuvre demi-tour", 3:"Navigation SLAM"};
const MODE_CLS    = {0:"m0", 1:"m1", 2:"m2", 3:"m3"};

function pill(id, active, labelOk, labelOff, isWarn) {
  const el = document.getElementById(id);
  if (active) {
    el.className = 'pill ' + (isWarn ? 'warn' : 'ok');
    el.textContent = labelOk;
  } else {
    el.className = 'pill off';
    el.textContent = labelOff;
  }
}

function setVal(id, v, decimals=0) {
  document.getElementById(id).textContent =
    typeof decimals === 'number' ? v.toFixed(decimals) : v;
}

function updateServo(angle) {
  const totalArc = 220;
  document.getElementById('servo-arc-fill').style.strokeDashoffset = totalArc * (1 - angle/180);
  document.getElementById('servo-needle').style.transform = `rotate(${angle - 90}deg)`;
  document.getElementById('servo-text').textContent = Math.round(angle) + '°';
}

// ---------- TF Tree ----------
function renderTF(transforms) {
  const grid = document.getElementById('tf-grid');
  const keys = Object.keys(transforms);
  if (keys.length === 0) {
    grid.innerHTML = '<span style="color:var(--dim);font-size:11px;font-style:italic;">En attente des TFs...</span>';
    return;
  }
  // Trier : statiques d'abord, puis par nom
  keys.sort((a, b) => {
    const as = transforms[a].static, bs = transforms[b].static;
    if (as && !bs) return -1;
    if (!as && bs) return 1;
    return a.localeCompare(b);
  });
  grid.innerHTML = keys.map(k => {
    const t = transforms[k];
    let cls = t.static ? 'static' : (t.fresh ? 'fresh' : 'stale');
    let ageStr, ageCls;
    if (t.static) {
      ageStr = 'STATIC'; ageCls = 'static';
    } else if (t.age_ms < 0) {
      ageStr = '?ms'; ageCls = 'fresh';
    } else {
      ageStr = t.age_ms + 'ms';
      ageCls = t.fresh ? 'fresh' : 'stale';
    }
    return `<div class="tf-item ${cls}">
      <span class="tf-name">${k}</span>
      <span class="tf-age ${ageCls}">${ageStr}</span>
    </div>`;
  }).join('');
}

// ---------- Topic Health ----------
const TOPIC_EXPECTED_HZ = {
  "/scan":              10,
  "/odom":              20,
  "/imu/data":          50,
  "/cmd_vel":           10,
  "/odometry/filtered": 20,
  "/cmd_vel_smoothed":  10,
};

function renderHealth(health) {
  const grid = document.getElementById('health-grid');
  grid.innerHTML = Object.entries(health).map(([topic, h]) => {
    const ok = h.fresh && h.hz > 0.5;
    const cardCls = ok ? 'ok' : 'dead';
    const hzCls   = ok ? 'ok' : 'dead';
    const dotCls  = ok ? 'ok' : 'dead';
    const expected = TOPIC_EXPECTED_HZ[topic] || '?';
    return `<div class="health-card ${cardCls}">
      <div class="health-topic">
        <span class="health-dot ${dotCls}"></span>${topic}
      </div>
      <span class="health-hz ${hzCls}">${h.hz.toFixed(1)}</span>
      <span class="health-unit">Hz</span>
      <div style="font-size:9px;color:var(--dim);margin-top:3px;">cible ~${expected} Hz</div>
    </div>`;
  }).join('');
}

// ---------- System Stats ----------
function renderSystem(d) {
  const cpuPct  = d.cpu_pct  || 0;
  const ramPct  = d.ram_pct  || 0;
  const diskPct = d.disk_pct || 0;

  const cpuEl = document.getElementById('cpu-pct');
  cpuEl.textContent = cpuPct.toFixed(1);
  cpuEl.style.color = cpuPct > 80 ? 'var(--red)' : cpuPct > 60 ? 'var(--orange)' : 'var(--green)';
  const cpuBar = document.getElementById('cpu-bar');
  cpuBar.style.width = cpuPct + '%';
  cpuBar.style.background = cpuPct > 80 ? 'var(--red)' : cpuPct > 60 ? 'var(--orange)' : 'var(--green)';

  const ramEl = document.getElementById('ram-pct');
  ramEl.textContent = ramPct.toFixed(1);
  ramEl.style.color = ramPct > 85 ? 'var(--red)' : ramPct > 65 ? 'var(--orange)' : 'var(--accent)';
  document.getElementById('ram-detail').textContent =
    `${d.ram_used_mb || 0} / ${d.ram_total_mb || 0} MB`;
  const ramBar = document.getElementById('ram-bar');
  ramBar.style.width = ramPct + '%';
  ramBar.style.background = ramPct > 85 ? 'var(--red)' : ramPct > 65 ? 'var(--orange)' : 'var(--accent)';

  const diskEl = document.getElementById('disk-pct');
  diskEl.textContent = diskPct.toFixed(1);
  diskEl.style.color = diskPct > 90 ? 'var(--red)' : diskPct > 70 ? 'var(--orange)' : 'var(--purple)';
  document.getElementById('disk-detail').textContent =
    `${d.disk_used_gb || 0} / ${d.disk_total_gb || 0} GB`;
  const diskBar = document.getElementById('disk-bar');
  diskBar.style.width = diskPct + '%';
  diskBar.style.background = diskPct > 90 ? 'var(--red)' : diskPct > 70 ? 'var(--orange)' : 'var(--purple)';
}

// ---------- Logs ----------
function renderLogs(logs) {
  const list = document.getElementById('log-list');
  if (!logs || logs.length === 0) {
    list.innerHTML = '<div class="log-empty">Aucun log WARN/ERROR pour l\\'instant</div>';
    return;
  }
  // Plus récent en haut
  const sorted = [...logs].reverse();
  list.innerHTML = sorted.map(l => {
    const t = new Date(l.ts * 1000).toLocaleTimeString('fr-FR', {hour12:false});
    return `<div class="log-entry ${l.level}">
      <span class="log-level ${l.level}">${l.level}</span>
      <span class="log-name" title="${l.name}">${l.name}</span>
      <span class="log-msg">${l.msg}</span>
      <span style="font-size:9px;color:var(--dim);white-space:nowrap;margin-left:8px;">${t}</span>
    </div>`;
  }).join('');
}

// ---------- Nodes Status ----------
const NODE_LABELS = {
  "/kiss_icp_node":      "KISS-ICP",
  "/ekf_filter_node":    "EKF",
  "/sllidar_node":       "LiDAR",
  "/orchestrateur_node": "Orchestrateur",
  "/node_controller":    "Controller",
  "/controller_server":  "Nav2 Controller",
  "/planner_server":     "Nav2 Planner",
  "/amcl":               "AMCL",
  "/node_camera":        "Camera",
  "/node_xbee":          "XBee",
};

function renderNodes(nodes) {
  const grid = document.getElementById('nodes-grid');
  // On affiche TOUJOURS tous les nodes, running ou pas
  const allNodes = Object.keys(NODE_LABELS);
  grid.innerHTML = allNodes.map(name => {
    const n = nodes[name] || { running: false };
    const cls = n.running ? 'up' : 'down';
    const label = NODE_LABELS[name];
    const status = n.running ? 'RUNNING' : 'OFFLINE';
    return `<div class="node-card ${cls}">
      <span class="node-dot ${cls}"></span>
      <div style="display:flex;flex-direction:column;overflow:hidden;">
        <span class="node-label" title="${name}">${label}</span>
        <span style="font-size:9px;letter-spacing:1px;color:${n.running ? 'var(--green)' : 'var(--red)'};">${status}</span>
      </div>
    </div>`;
  }).join('');
}

// ---------- AMCL Pose ----------
function renderAmcl(d) {
  if (d.amcl_ts === 0) {
    ['amcl-x','amcl-y','amcl-yaw'].forEach(id =>
      document.getElementById(id).textContent = '—');
    document.getElementById('amcl-ts').textContent = 'Aucune pose reçue';
    return;
  }
  document.getElementById('amcl-x').textContent   = d.amcl_x.toFixed(3);
  document.getElementById('amcl-y').textContent   = d.amcl_y.toFixed(3);
  document.getElementById('amcl-yaw').textContent = d.amcl_yaw.toFixed(1) + '°';
  const dt = new Date(d.amcl_ts * 1000);
  document.getElementById('amcl-ts').textContent =
    dt.toLocaleTimeString('fr-FR', {hour12:false});
}

// ---------- Poll ----------
async function poll() {
  try {
    const r = await fetch('/data');
    const d = await r.json();

    // Connexion
    const fresh = (Date.now()/1000 - d.ts) < 3;
    const dot = document.getElementById('conn-dot');
    const lbl = document.getElementById('conn-label');
    dot.className = fresh ? 'live' : '';
    lbl.textContent = fresh ? 'LIVE' : 'STALE';
    lbl.style.color = fresh ? 'var(--green)' : 'var(--red)';

    // Mode
    const m = d.robot_mode;
    const badge = document.getElementById('mode-badge');
    badge.className = 'mode-badge ' + (MODE_CLS[m] || 'm0');
    badge.textContent = MODE_LABELS[m] || '?';
    document.getElementById('mode-sub').textContent = MODE_SUBS[m] || '';

    // Flags
    pill('pill-course', d.course_active,    'COURSE ACTIVE',  'COURSE INACTIVE', false);
    pill('pill-sens',   d.bon_sens,         'BON SENS',       'MAUVAIS SENS',    false);
    pill('pill-obs',    d.obstacle_arriere, 'OBS ARRIÈRE !!', 'OBS ARRIÈRE',     true);
    document.getElementById('sens-dt').textContent = d.sens_demi_tour || '—';

    // IR
    const irG = document.getElementById('ir-g');
    const irD = document.getElementById('ir-d');
    irG.textContent = d.ir_gauche_mm.toFixed(0);
    irD.textContent = d.ir_droit_mm.toFixed(0);
    irG.className = 'val' + (d.ir_gauche_mm < 150 ? ' hi' : '');
    irD.className = 'val' + (d.ir_droit_mm  < 150 ? ' hi' : '');

    // cmd_vel
    setVal('vel-lin', d.cmd_vel_linear,  3);
    setVal('vel-ang', d.cmd_vel_angular, 3);
    const velPct = Math.min(Math.abs(d.cmd_vel_linear) / 0.5 * 100, 100);
    const velBar = document.getElementById('vel-bar');
    velBar.style.width = velPct + '%';
    velBar.style.background = d.cmd_vel_linear >= 0 ? 'var(--green)' : 'var(--red)';

    // Servo + IMU
    updateServo(d.servo_angle);
    setVal('imu-yaw', d.imu_yaw_deg, 1);
    document.getElementById('compass-needle').style.transform = `rotate(${d.imu_yaw_deg}deg)`;

    // LiDAR
    const frontEl = document.getElementById('lid-front');
    frontEl.textContent = d.lidar_front_min.toFixed(3);
    frontEl.className = 'val' + (d.lidar_front_min < 0.5 && d.lidar_front_min > 0 ? ' hi' : '');
    setVal('lid-left',  d.lidar_left_avg,  3);
    setVal('lid-right', d.lidar_right_avg, 3);

    // Nouveaux panels
    renderTF(d.tf_transforms || {});
    renderHealth(d.topic_health || {});
    renderSystem(d);
    renderLogs(d.rosout_logs || []);
    renderNodes(d.nodes_status || {});
    renderAmcl(d);

    // Timestamp
    const dt = new Date(d.ts * 1000);
    document.getElementById('ts-label').textContent =
      'Dernière maj : ' + dt.toLocaleTimeString('fr-FR', {hour12:false});

  } catch(e) {
    document.getElementById('conn-dot').className = '';
    document.getElementById('conn-label').textContent = 'OFFLINE';
    document.getElementById('conn-label').style.color = 'var(--red)';
  }
}

setInterval(poll, 250);
// Afficher les nodes OFFLINE immédiatement sans attendre le poll
renderNodes({});
poll();
</script>
</body>
</html>"""


# ---------------------------------------------------------------------------
# Serveur HTTP
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == "/data":
            with state_lock:
                payload = json.dumps(state).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(payload)

        elif self.path in ("/", "/index.html"):
            body = DASHBOARD_HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()


def run_http(port=8080):
    httpd = HTTPServer(("0.0.0.0", port), Handler)
    print(f"[monitor] Dashboard sur http://localhost:{port}")
    httpd.serve_forever()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = MonitorNode()

    t = threading.Thread(target=run_http, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
