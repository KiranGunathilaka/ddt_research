#!/usr/bin/env bash
# desc: run the simulator (subcommands: run)

set -euo pipefail

SUBCMD="${1:-help}"
shift || true

case "$SUBCMD" in
    run)
        set +u; source /opt/ros/humble/setup.bash; set -u
        cd "$EUFS_MASTER"

        if [[ ! -f install/setup.bash ]]; then
            echo "bri sim run: workspace is not built yet." >&2
            echo "Run 'bri build' first." >&2
            exit 1
        fi
        set +u; source install/setup.bash; set -u

        # Restart foxglove_bridge (kill stale instance if any, then relaunch)
        pkill -x foxglove_bridge 2>/dev/null || true
        nohup ros2 run foxglove_bridge foxglove_bridge \
            --ros-args -p port:=8765 \
            > /tmp/foxglove_bridge.log 2>&1 &
        echo "foxglove_bridge started on ws://localhost:8765 (log: /tmp/foxglove_bridge.log)"

        exec ros2 launch eufs_sim2 eufs_sim2.launch.py "$@"
        ;;
    -h|--help|help|"")
        cat <<EOF
bri sim — simulator control

Usage:
  bri sim run [launch_args...]   Launch eufs_sim2.launch.py

The foxglove_bridge is auto-started by the dev container on ws://localhost:8765;
no need to launch it manually. Tail its log with:
    tail -f /tmp/foxglove_bridge.log
EOF
        ;;
    *)
        echo "bri sim: unknown subcommand '$SUBCMD'" >&2
        exit 1
        ;;
esac
