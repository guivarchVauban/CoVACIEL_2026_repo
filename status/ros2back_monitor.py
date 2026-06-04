#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════╗
║           ROS2 DOCKER MONITOR  🤖🐳                     ║
║  Watch-style dashboard — containers + nodes ROS2        ║
╚══════════════════════════════════════════════════════════╝

Usage:
    python3 ros2_monitor.py              # refresh toutes les 3s
    python3 ros2_monitor.py --interval 5 # refresh custom
    python3 ros2_monitor.py --no-log     # sans fichier log

Dépendances : pip install rich
"""

import subprocess
import time
import json
import sys
import argparse
from datetime import datetime
from pathlib import Path

try:
    from rich.console import Console, Group
    from rich.table import Table
    from rich.live import Live
    from rich.panel import Panel
    from rich.text import Text
    from rich.align import Align
    from rich.columns import Columns
    from rich.rule import Rule
    from rich import box
    from rich.padding import Padding
    from rich.spinner import Spinner
    from rich.progress_bar import ProgressBar
except ImportError:
    print("❌ Module 'rich' manquant. Installe-le avec : pip install rich")
    sys.exit(1)

# ─────────────────────────────────────────────
#  Configuration
# ─────────────────────────────────────────────

DEFAULT_INTERVAL = 15       # secondes entre chaque refresh
LOG_FILE         = Path("ros2_monitor.log")
ROS_SETUP_CMD    = "source /opt/ros/$(ls /opt/ros/ 2>/dev/null | head -1)/setup.bash 2>/dev/null"

# ─────────────────────────────────────────────
#  Nodes critiques à surveiller par container
#  Clé = sous-chaîne du nom du container (case-insensitive)
#  Si une node de cette liste est absente → affichée ÉTEINT
# ─────────────────────────────────────────────
WATCHED_NODES: dict[str, list[str]] = {
    "eliot_jazzy": [
        "/imu_filter_madgwick",
        "/imu_node",
        "/nav_tunnel_pid_node",
        "/node_camera",
        "/node_controller",
        "/orchestrateur_node",
        "/sllidar_node",
    ],
    # Ajoute d'autres containers ici si besoin :
    # "eliot_mapping": ["/kiss_icp_node", "/laser_scan_to_pc"],
}

# Icônes par statut
STATUS_ICONS = {
    "ACTIF":        ("●", "bold bright_green"),
    "REDÉMARRAGE":  ("◎", "bold yellow"),
    "ÉTEINT":       ("○", "bold red"),
    "EN PAUSE":     ("⏸", "bold cyan"),
    "CRÉÉ":         ("◌", "bold blue"),
    "INCONNU":      ("?", "dim white"),
}

console = Console()

# ─────────────────────────────────────────────
#  Docker helpers
# ─────────────────────────────────────────────

def run(cmd: list, timeout: int = 10) -> tuple[int, str, str]:
    """Lance une commande, retourne (returncode, stdout, stderr)."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout.strip(), r.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "TIMEOUT"
    except FileNotFoundError:
        return -1, "", "COMMAND_NOT_FOUND"
    except Exception as e:
        return -1, "", str(e)


def get_docker_containers() -> list[dict]:
    """Liste tous les containers Docker (running ou non)."""
    code, out, _ = run([
        "docker", "ps", "-a",
        "--format", '{"id":"{{.ID}}","name":"{{.Names}}","image":"{{.Image}}",'
                    '"status":"{{.Status}}","state":"{{.State}}","ports":"{{.Ports}}"}'
    ])
    if code != 0 or not out:
        return []
    containers = []
    for line in out.splitlines():
        line = line.strip()
        if line:
            try:
                containers.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return containers


def map_docker_state(state: str) -> tuple[str, str]:
    """Mappe l'état Docker → (label_fr, style_rich)."""
    s = (state or "").lower()
    if "running"    in s: return "ACTIF",       "bright_green"
    if "restarting" in s: return "REDÉMARRAGE", "yellow"
    if "exited"     in s: return "ÉTEINT",      "red"
    if "dead"       in s: return "ÉTEINT",      "red"
    if "paused"     in s: return "EN PAUSE",    "cyan"
    if "created"    in s: return "CRÉÉ",        "blue"
    return "INCONNU", "white"


# ─────────────────────────────────────────────
#  ROS2 helpers
# ─────────────────────────────────────────────

def get_ros2_nodes(container_id: str) -> list[str]:
    """Récupère la liste des nodes ROS2 dans un container."""
    code, out, _ = run([
        "docker", "exec", container_id,
        "bash", "-c",
        f"{ROS_SETUP_CMD} && ros2 node list 2>/dev/null"
    ], timeout=8)
    if code == 0 and out:
        return [n for n in out.splitlines() if n.strip().startswith("/")]
    return []


def check_node_alive(container_id: str, node: str) -> tuple[str, str]:
    """Vérifie qu'un node ROS2 répond via ros2 node info."""
    code, out, _ = run([
        "docker", "exec", container_id,
        "bash", "-c",
        f"{ROS_SETUP_CMD} && ros2 node info {node} 2>/dev/null | head -3"
    ], timeout=6)
    if code == -1:
        return "REDÉMARRAGE", "yellow"   # timeout → probablement en train de redémarrer
    if code == 0 and out:
        return "ACTIF", "bright_green"
    return "ÉTEINT", "red"


# ─────────────────────────────────────────────
#  Collecte
# ─────────────────────────────────────────────

def get_watched_nodes_for(container_name: str) -> list[str]:
    """Retourne les nodes critiques configurées pour ce container (matching partiel)."""
    name_lower = container_name.lower()
    for key, nodes in WATCHED_NODES.items():
        if key.lower() in name_lower:
            return nodes
    return []


def collect() -> list[dict]:
    """Collecte l'état de tous les containers et leurs nodes ROS2."""
    raw = get_docker_containers()
    result = []
    for c in raw:
        c_name = c.get("name", "unknown")
        label, style = map_docker_state(c.get("state", ""))
        nodes = []

        if label == "ACTIF":
            # Nodes détectées dynamiquement
            discovered = set(get_ros2_nodes(c["id"]))

            # Nodes critiques configurées pour ce container
            watched = get_watched_nodes_for(c_name)

            # Ordre : nodes critiques d'abord, puis les autres triées
            all_nodes_ordered = list(watched) + [n for n in sorted(discovered) if n not in watched]

            for node_name in all_nodes_ordered:
                is_watched = node_name in watched
                if node_name not in discovered:
                    # Node attendue mais absente → ÉTEINT immédiat, pas besoin d'exec
                    nodes.append({
                        "name":    node_name,
                        "status":  "ÉTEINT",
                        "style":   "red",
                        "watched": is_watched,
                    })
                else:
                    n_label, n_style = check_node_alive(c["id"], node_name)
                    nodes.append({
                        "name":    node_name,
                        "status":  n_label,
                        "style":   n_style,
                        "watched": is_watched,
                    })

        result.append({
            "id":     c["id"],
            "name":   c_name,
            "image":  c.get("image", "unknown"),
            "ports":  c.get("ports", "") or "—",
            "status_raw": c.get("state", ""),
            "label":  label,
            "style":  style,
            "nodes":  nodes,
        })
    return result


# ─────────────────────────────────────────────
#  Rendu Rich
# ─────────────────────────────────────────────

def status_badge(label: str, style: str) -> Text:
    icon, icon_style = STATUS_ICONS.get(label, ("?", "dim"))
    t = Text()
    t.append(f" {icon} ", style=icon_style)
    t.append(label, style=f"bold {style}")
    t.append(" ")
    return t


def build_header(now: str, interval: int) -> Panel:
    title = Text(justify="center")
    title.append("🤖  ROS2 DOCKER MONITOR  🐳\n", style="bold bright_cyan")
    title.append(f"  {now}  ", style="dim white")
    title.append(f"  ⟳ {interval}s  ", style="dim cyan")
    return Panel(
        Align.center(title, vertical="middle"),
        box=box.DOUBLE_EDGE,
        border_style="bright_cyan",
        height=5,
    )


def build_table(data: list[dict]) -> Table:
    t = Table(
        box=box.ROUNDED,
        border_style="cyan",
        header_style="bold magenta",
        show_lines=True,
        padding=(0, 1),
        expand=True,
    )
    t.add_column("🐳  Container",      style="bold white",      min_width=18, no_wrap=True)
    t.add_column("📦  Image",          style="dim white",       min_width=22)
    t.add_column("🔌  Ports",          style="dim cyan",        min_width=16)
    t.add_column("⚙️   Docker",         justify="center",        min_width=16)
    t.add_column("🤖  Node ROS2",      style="bold white",      min_width=28, no_wrap=True)
    t.add_column("💡  Status Node",    justify="center",        min_width=16)

    for c in data:
        docker_badge = status_badge(c["label"], c["style"])

        # Tronque l'image si trop longue
        img = c["image"]
        if len(img) > 28:
            img = img[:25] + "…"

        # Tronque les ports
        ports = c["ports"]
        if len(ports) > 22:
            ports = ports[:20] + "…"

        if not c["nodes"]:
            t.add_row(
                c["name"], img, ports,
                docker_badge,
                Text("— aucune node ROS2 —", style="dim italic"),
                Text("—", style="dim"),
            )
        else:
            for i, node in enumerate(c["nodes"]):
                node_badge = status_badge(node["status"], node["style"])
                # Indicateur visuel pour les nodes critiques configurées
                node_label = Text()
                if node.get("watched"):
                    node_label.append("⚑ ", style="bold yellow")
                node_label.append(node["name"], style="bold white")

                if i == 0:
                    t.add_row(c["name"], img, ports, docker_badge, node_label, node_badge)
                else:
                    t.add_row("", "", "", Text(""), node_label, node_badge)

    return t


def build_footer(data: list[dict]) -> Panel:
    total_c  = len(data)
    active_c = sum(1 for c in data if c["label"] == "ACTIF")
    restart_c= sum(1 for c in data if c["label"] == "REDÉMARRAGE")

    all_nodes   = [n for c in data for n in c["nodes"]]
    total_n     = len(all_nodes)
    active_n    = sum(1 for n in all_nodes if n["status"] == "ACTIF")
    restart_n   = sum(1 for n in all_nodes if n["status"] == "REDÉMARRAGE")
    dead_n      = total_n - active_n - restart_n

    t = Text(justify="center")
    t.append("Containers :  ", style="dim")
    t.append(f"✅ {active_c} actifs", style="bright_green")
    t.append("  ")
    t.append(f"🔄 {restart_c} redémarrages", style="yellow")
    t.append(f"  ⬜ {total_c - active_c - restart_c} éteints", style="red")
    t.append("     |     ", style="dim")
    t.append("Nodes ROS2 :  ", style="dim")
    t.append(f"✅ {active_n} actives", style="bright_green")
    t.append("  ")
    t.append(f"🔄 {restart_n} redémarrages", style="yellow")
    t.append(f"  ⬜ {dead_n} éteintes", style="red")
    t.append("     |     ", style="dim")
    t.append("Ctrl+C  pour quitter", style="dim italic")

    return Panel(t, box=box.SIMPLE, border_style="dim cyan")


def build_no_docker_panel() -> Panel:
    return Panel(
        Align.center(
            Text("⚠️  Docker introuvable ou aucun container détecté.\n"
                 "Vérifie que Docker tourne et que tu as les droits.",
                 style="bold yellow", justify="center"),
            vertical="middle"
        ),
        border_style="yellow",
        height=5,
    )


def build_dashboard(data: list[dict], now: str, interval: int):
    header = build_header(now, interval)
    if not data:
        return Group(header, build_no_docker_panel())
    table  = build_table(data)
    footer = build_footer(data)
    return Group(header, table, footer)


# ─────────────────────────────────────────────
#  Logging fichier
# ─────────────────────────────────────────────

def write_log(data: list[dict], now: str):
    with LOG_FILE.open("a") as f:
        f.write(f"\n{'━'*60}\n")
        f.write(f"[{now}]\n")
        for c in data:
            f.write(f"  CONTAINER  {c['name']:<25} [{c['label']}]  ({c['image']})\n")
            if not c["nodes"]:
                f.write("    └─ aucune node ROS2 détectée\n")
            for i, node in enumerate(c["nodes"]):
                prefix = "└─" if i == len(c["nodes"]) - 1 else "├─"
                f.write(f"    {prefix} NODE  {node['name']:<40} [{node['status']}]\n")


# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="ROS2 Docker Monitor")
    p.add_argument("--interval", "-i", type=int, default=DEFAULT_INTERVAL,
                   help="Intervalle de refresh en secondes (défaut: 3)")
    p.add_argument("--no-log", action="store_true",
                   help="Désactive l'écriture du fichier log")
    return p.parse_args()


def main():
    args = parse_args()
    interval = max(1, args.interval)
    log_enabled = not args.no_log

    console.print()
    console.print(Panel(
        "[bold bright_cyan]🚀  Démarrage du moniteur ROS2 Docker[/bold bright_cyan]\n"
        f"[dim]Refresh : {interval}s  |  Log : {'[green]activé[/green]' if log_enabled else '[red]désactivé[/red]'}[/dim]",
        box=box.ROUNDED, border_style="cyan"
    ))
    console.print()

    # Premier collecte avec message d'attente
    console.print("[dim cyan]⟳  Collecte initiale des données…[/dim cyan]")
    time.sleep(0.5)

    iteration = 0
    with Live(console=console, refresh_per_second=2, screen=True) as live:
        while True:
            try:
                now  = datetime.now().strftime("%Y-%m-%d  %H:%M:%S")
                data = collect()
                dashboard = build_dashboard(data, now, interval)
                live.update(dashboard)

                if log_enabled:
                    write_log(data, now)

                iteration += 1
                time.sleep(interval)

            except KeyboardInterrupt:
                break

    console.print()
    console.print(Panel(
        f"[bold yellow]👋  Moniteur arrêté après {iteration} itération(s).[/bold yellow]\n"
        + (f"[dim]Log sauvegardé dans [cyan]{LOG_FILE.resolve()}[/cyan][/dim]" if log_enabled else ""),
        box=box.ROUNDED, border_style="yellow"
    ))
    console.print()


if __name__ == "__main__":
    main()
