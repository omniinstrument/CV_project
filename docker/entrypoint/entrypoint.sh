#!/bin/bash

WS="/home/$(whoami)/ros2_ws"

is_built() {
    [[ -d "${WS}/install" ]]
}

build_ws() {
    echo "[entrypoint] Building ROS 2 workspace..."
    cd "${WS}"
    source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) && cd
}

source_ws() {
    local setup="${WS}/install/local_setup.bash"
    if [[ -f "$setup" ]]; then
        echo "[entrypoint] Sourcing workspace."
        # shellcheck disable=SC1090
        source "$setup"
    fi
}

echo "[entrypoint] Starting entrypoint as user: $(whoami)"

if ! is_built; then
    build_ws
else
    echo "[entrypoint] Workspace already built. Skipping."
fi

source_ws

echo "[entrypoint] Executing command: $*"
exec "$@"
