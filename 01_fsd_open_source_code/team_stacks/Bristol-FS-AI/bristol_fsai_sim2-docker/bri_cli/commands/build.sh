#!/usr/bin/env bash
# desc: colcon build the workspace (optionally selected packages)

set -euo pipefail

set +u; source /opt/ros/humble/setup.bash; set -u
cd "$EUFS_MASTER"

if [[ $# -eq 0 ]]; then
    colcon build --symlink-install
else
    colcon build --symlink-install --packages-select "$@"
fi

echo ""
echo "Build done. New shells auto-source install/setup.bash; in this shell run:"
echo "    source install/setup.bash"
