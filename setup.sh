#!/bin/bash
# Set up the Wall Follow Dashboard + service on a stock Neoracer.
#
# From your laptop, on the same network as the car:
#   scp -r neoracer_wallfollow racecar@<car-ip>:~/wallfollow
#   ssh racecar@<car-ip> 'bash ~/wallfollow/setup.sh'
#
# Installs neoracer-wallfollow.service, enables it at boot, and starts it.
# Dashboard comes up at http://<car-ip>:8081. Idempotent: safe to re-run.
#
# Subcommands:
#   (none)    install or update, then start
#   restart   restart the service, taking port 8081 back first
#   remove    stop, disable, and uninstall the unit; tuning yaml is kept
#   disable   stop and keep off across boots
#   enable    turn back on and start
#
# Needs nothing beyond the stock image: ROS Humble at /opt/ros/humble, the
# driver workspace at /home/racecar/ros2_ws, and the racecar user.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST=/home/racecar/wallfollow
SVC=neoracer-wallfollow.service
PORT=8081

# PIDs listening on $PORT. Unprivileged ss names only our own processes, so
# fall back to sudo when a listener exists but its owner stays hidden.
port_pids() {
    local out
    out="$(ss -ltnpH "sport = :$PORT" 2>/dev/null || true)"
    if [[ -n "$out" && "$out" != *pid=* ]]; then
        out="$(sudo ss -ltnpH "sport = :$PORT" 2>/dev/null || true)"
    fi
    grep -o 'pid=[0-9]*' <<<"$out" | cut -d= -f2 | sort -u || true
}

# Service unit a PID belongs to, empty for a process started by hand.
unit_of() {
    sed -n 's|.*/\([^/]*\.service\).*|\1|p' "/proc/$1/cgroup" 2>/dev/null | head -1
}

# sudo only for processes we do not own.
signal_pid() {
    if [[ "$(stat -c %u "/proc/$1" 2>/dev/null)" == "$(id -u)" ]]; then
        kill "$2" "$1" 2>/dev/null || true
    else
        sudo kill "$2" "$1" 2>/dev/null || true
    fi
}

# PIDs on $PORT that are not this service.
foreign_pids() {
    local pid
    for pid in $(port_pids); do
        [[ "$(unit_of "$pid")" == "$SVC" ]] || echo "$pid"
    done
}

# Clear anything else off $PORT: an earlier run of this dashboard under a
# different unit name, a sibling dashboard, or a hand-started wallfollow.py.
# Killing a service-owned PID would only trip its own Restart=on-failure, so
# stop the owning unit instead.
free_port() {
    local pid unit
    for pid in $(foreign_pids); do
        unit="$(unit_of "$pid")"
        if [[ -n "$unit" ]]; then
            echo "  port $PORT held by $unit: stopping it"
            sudo systemctl stop "$unit" || true
        else
            echo "  port $PORT held by pid $pid started outside systemd: stopping it"
            signal_pid "$pid" -TERM
        fi
    done
    for _ in $(seq 20); do
        [[ -z "$(foreign_pids)" ]] && return 0
        sleep 0.5
    done
    for pid in $(foreign_pids); do
        [[ -n "$(unit_of "$pid")" ]] || signal_pid "$pid" -KILL
    done
    sleep 1
    [[ -z "$(foreign_pids)" ]] || echo "  warning: port $PORT is still busy"
}

case "${1:-}" in
    disable)
        sudo systemctl disable --now "$SVC"
        echo "$SVC stopped and disabled. It will not start at boot."
        echo "Re-enable with: bash setup.sh enable"
        exit 0
        ;;
    enable)
        free_port
        sudo systemctl enable --now "$SVC"
        echo "$SVC enabled and started."
        exit 0
        ;;
    restart)
        sudo systemctl stop "$SVC" 2>/dev/null || true
        free_port
        sudo systemctl restart "$SVC"
        sleep 3
        systemctl is-active --quiet "$SVC" || {
            echo "$SVC failed to start:"; systemctl status "$SVC" --no-pager | tail -5; exit 1; }
        echo "$SVC restarted: http://$(hostname -I | awk '{print $1}'):$PORT"
        exit 0
        ;;
    remove)
        sudo systemctl disable --now "$SVC" 2>/dev/null || true
        sudo rm -f "/etc/systemd/system/$SVC"
        sudo systemctl daemon-reload
        sudo systemctl reset-failed "$SVC" 2>/dev/null || true
        echo "$SVC removed. Files in $DEST are untouched."
        echo "Reinstall with: bash setup.sh"
        exit 0
        ;;
    "")
        ;;
    *)
        echo "usage: bash setup.sh [enable|disable|restart|remove]" >&2
        exit 2
        ;;
esac

# The service unit expects the files at $DEST. If the script runs from
# somewhere else, copy them over. Keep an existing tuning yaml.
if [[ "$SCRIPT_DIR" != "$DEST" ]]; then
    mkdir -p "$DEST"
    cp "$SCRIPT_DIR/wallfollow.py" "$SCRIPT_DIR/wallfollow.html" "$SCRIPT_DIR/$SVC" \
       "$SCRIPT_DIR/setup.sh" "$DEST/"
    [[ -f "$DEST/wallfollow.yaml" ]] || cp "$SCRIPT_DIR/wallfollow.yaml" "$DEST/"
fi

if ! cmp -s "$DEST/$SVC" "/etc/systemd/system/$SVC" 2>/dev/null; then
    sudo install -m 0644 "$DEST/$SVC" "/etc/systemd/system/$SVC"
    sudo systemctl daemon-reload
    echo "  $SVC: installed"
fi

sudo systemctl stop "$SVC" 2>/dev/null || true
free_port
sudo systemctl enable "$SVC" 2>/dev/null
sudo systemctl restart "$SVC"    # pick up file changes on re-run

sleep 3
systemctl is-active --quiet "$SVC" || { echo "$SVC failed to start:"; systemctl status "$SVC" --no-pager | tail -5; exit 1; }
echo "Wall Follow Dashboard running: http://$(hostname -I | awk '{print $1}'):$PORT"
