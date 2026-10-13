#!/bin/bash
# Build and run one app against MuJoCo inside the devcontainer.
# Usage: bash scripts/test_sim.sh [app-name]
# Override ROS_DOMAIN_ID (default 42) and HTTP_PORT (fixed port, default 23517) per developer.
# Set SCENE_EDITING=true to open the editor at / when starting a new simulator.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APP="${1:-}"
if [ $# -gt 1 ]; then
    echo "Usage: $0 [app-name]" >&2
    exit 1
fi
if [ -z "$APP" ]; then
    apps=()
    for path in "$ROOT"/apps/*/start.sh; do
        [ -f "$path" ] && apps+=("$(basename "$(dirname "$path")")")
    done
    [ ${#apps[@]} -gt 0 ] || { echo "No apps with start.sh found" >&2; exit 1; }
    echo "Select the app to run against MuJoCo:"
    select APP in "${apps[@]}"; do
        [ -n "$APP" ] && break
    done
fi
[[ "$APP" =~ ^[a-z0-9_-]+$ ]] && [ -f "$ROOT/apps/$APP/start.sh" ] || {
    echo "Unknown app: $APP (expected apps/<name>/start.sh)" >&2; exit 1;
}
[ -f /opt/ros/humble/setup.bash ] || {
    echo "Run this inside the ROS 2 Humble devcontainer" >&2; exit 1;
}
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export HTTP_PORT="${HTTP_PORT:-23517}"
export SCENE_EDITING="${SCENE_EDITING:-false}"
source /opt/ros/humble/setup.bash
WS="$(cd "$ROOT/../.." && pwd)"
cd "$WS"
colcon build --symlink-install \
    --base-paths "$ROOT/apps/$APP" "$ROOT/sim/piper_mujoco_sim"
source "$WS/install/setup.bash"
python3 - "$ROOT/apps/$APP" <<'PY'
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.request

from pathlib import Path
import errno

app_dir = sys.argv[1]
scene_editing = os.environ['SCENE_EDITING'].lower()
if scene_editing not in ('true', 'false'):
    sys.exit('SCENE_EDITING must be true or false')
try:
    port = int(os.environ['HTTP_PORT'])
    if not 1 <= port <= 65535:
        raise ValueError
except ValueError:
    sys.exit('HTTP_PORT must be an integer between 1 and 65535')


def listeners(port):
    """Find Linux socket owners without requiring ss or lsof."""
    inodes = set()
    for table in ('/proc/net/tcp', '/proc/net/tcp6'):
        for line in Path(table).read_text().splitlines()[1:]:
            fields = line.split()
            if fields[3] == '0A' and int(fields[1].split(':')[1], 16) == port:
                inodes.add(fields[9])
    owners = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            if not any(os.readlink(fd) in {f'socket:[{inode}]' for inode in inodes}
                       for fd in (process / 'fd').iterdir()):
                continue
            args = (process / 'cmdline').read_bytes().decode().strip('\0').split('\0')
            name = (process / 'comm').read_text().strip()
            env = dict(item.split('=', 1) for item in
                       (process / 'environ').read_bytes().decode().split('\0') if '=' in item)
            owners.append((int(process.name), name, args, env.get('ROS_DOMAIN_ID', '0')))
        except (OSError, UnicodeError):
            continue
    return owners


existing_pid = None
with socket.socket() as probe:
    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        probe.bind(('0.0.0.0', port))
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            sys.exit(f'Cannot use browser port {port}: {error}')
        owners = listeners(port)
        for pid, name, args, domain in owners:
            print(f'Port {port} is in use: {name}, PID {pid}: {" ".join(args)}', flush=True)
        if len(owners) != 1:
            sys.exit(f'Cannot identify a single simulator owner for port {port}. '
                     'Check the listener on the host; no new server was started.')
        pid, name, args, domain = owners[0]
        if not any(arg.endswith('/lib/piper_mujoco_sim/sim') for arg in args):
            sys.exit(f'Port {port} belongs to another process. Stop it or explicitly set HTTP_PORT.')
        if domain != os.environ['ROS_DOMAIN_ID']:
            sys.exit(f'Simulator PID {pid} uses ROS_DOMAIN_ID={domain}, '
                     f'but this app uses {os.environ["ROS_DOMAIN_ID"]}. '
                     f'Rerun with ROS_DOMAIN_ID={domain} to use that simulator.')
        existing_pid = pid
        print(f'Reusing MuJoCo simulator PID {pid} on port {port}.', flush=True)
processes = []


def interrupted(signum, frame):
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, interrupted)
signal.signal(signal.SIGTERM, interrupted)
try:
    sim = None
    if existing_pid is None:
        sim = subprocess.Popen(
            ['ros2', 'launch', 'piper_mujoco_sim', 'sim.launch.py', f'http_port:={port}',
             f'scene_editing:={scene_editing}'],
            start_new_session=True)
        processes.append(sim)
    deadline = time.monotonic() + 30
    while True:
        if sim is not None and sim.poll() is not None:
            raise RuntimeError('Simulator exited before startup')
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/', timeout=1) as response:
                if response.status == 200 and b'<title>PIPER MuJoCo</title>' in response.read():
                    break
        except OSError:
            pass
        if time.monotonic() >= deadline:
            raise RuntimeError('Simulator did not start within 30 seconds')
        time.sleep(0.2)
    print(f'\nBrowser: http://localhost:{port} (remote: http://<dev-pc-ip>:{port})', flush=True)
    print(f'Scene editor: http://localhost:{port}/edit', flush=True)
    stop_message = 'Ctrl+C stops the app; the existing simulator stays running.' if existing_pid else 'Ctrl+C stops both.'
    print(f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}; starting {os.path.basename(app_dir)}. {stop_message}\n", flush=True)
    app = subprocess.Popen(['bash', 'start.sh'], cwd=app_dir, start_new_session=True)
    processes.append(app)
    while (sim is None or sim.poll() is None) and app.poll() is None:
        time.sleep(0.2)
    if sim is not None and sim.poll() is not None:
        raise RuntimeError(f'Simulator exited with status {sim.returncode}')
    sys.exit(app.returncode)
except KeyboardInterrupt:
    pass
finally:
    # Signal each launch process; ROS launch forwards the signal to its nodes.
    for process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    for process in reversed(processes):
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
PY
