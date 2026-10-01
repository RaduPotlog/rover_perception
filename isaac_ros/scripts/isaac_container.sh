#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# Run the Isaac ROS dev container without a terminal (scripts, CI, agents). Run in the Isaac
# ROS distro (Ubuntu-24.04). Interactive use needs none of this: just `isaac-ros activate`.
#
#   isaac_container.sh start            # same container as `isaac-ros activate`, detached
#   isaac_container.sh exec <command>   # run a shell command in it as `admin`, ROS sourced,
#                                       # on rmw_zenoh (router 127.0.0.1:7448, started by the image)
#   isaac_container.sh stop
#
# `isaac-ros activate` runs `docker run -it --rm`, which refuses to start without a TTY, so
# `start` gives it a pseudo-terminal with script(1). Attach the camera BEFORE `start`: the
# container only sees USB devices that exist when it is created.
set -euo pipefail

NAME=isaac_ros_dev_container
export ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev/}"

running() { [[ -n "$(docker ps -q --filter "name=^${NAME}$")" ]]; }

case "${1:-}" in
    start)
        if running; then
            echo "$NAME already running"
            exit 0
        fi
        nohup script -qfc 'isaac-ros activate --start-only' /dev/null \
            > "$HOME/isaac_ros_container.log" 2>&1 < /dev/null &
        disown
        # The entrypoint creates the `admin` user before it hands over to bash.
        for _ in $(seq 60); do
            if running && docker exec "$NAME" id admin > /dev/null 2>&1; then
                echo "$NAME started"
                exit 0
            fi
            sleep 1
        done
        echo "container did not start; see ~/isaac_ros_container.log" >&2
        tail -n 20 "$HOME/isaac_ros_container.log" >&2
        exit 1
        ;;
    exec)
        shift
        running || { echo "$NAME is not running: $0 start" >&2; exit 1; }
        exec docker exec -u admin -w /workspaces/isaac_ros-dev "$NAME" \
            bash -c "source /opt/ros/lyrical/setup.bash && source /etc/profile.d/50-rover-zenoh.sh && $*"
        ;;
    stop)
        docker stop "$NAME" > /dev/null 2>&1 || true
        ;;
    *)
        sed -n '5,15p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
