#!/usr/bin/env bash
# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.
#
# What does the perception stack cost on THIS machine? Launches rover_perception_bringup (and,
# with use_camera:=true, the RealSense driver from rover_sensors' rover_realsense), lets it warm
# up, then reports per-process CPU (percent of ONE core, so 100 = a full core) and memory, plus
# the real topic rates. Run the same command on the laptop and on the rover and compare.
#
#   benchmark.sh [seconds] [extra launch args...]
#   benchmark.sh 60 use_camera:=true use_fiducials:=true use_terrain:=true
#   benchmark.sh 60 use_camera:=true camera_fps:=10 depth_profile:=320x180
#   benchmark.sh 60 use_camera:=true use_person_tracking:=true
#
# camera_fps, depth_profile and use_depth_cloud go to the driver, the rest to perception.
#
# Needs a sourced ROS 2 workspace and a running Zenoh router when rmw_zenoh_cpp is the RMW.
# Run the rest of the rover's stacks at the same time to see the real contention; the CPU
# columns only count the perception processes, so also read the "machine" line.
set -euo pipefail

DURATION="${1:-60}"
shift || true
ARGS=("$@")
[[ ${#ARGS[@]} -eq 0 ]] && ARGS=(use_camera:=true)
WARMUP=15
PATTERNS=(realsense2_camera_node component_container apriltag_node terrain_node detection_node
          fmoc_node)
TOPICS=(camera/color/image_raw camera/depth/image_rect_raw camera/depth/points tracked_person)
NS="${ROVER_SYSTEM_NAMESPACE:-}"
TICKS="$(getconf CLK_TCK)"

DRIVER_ARGS=()
PERCEPTION_ARGS=()
USE_CAMERA=false
for a in "${ARGS[@]}"; do
    case "$a" in
        camera_fps:=* | depth_profile:=* | use_depth_cloud:=*) DRIVER_ARGS+=("$a") ;;
        *) PERCEPTION_ARGS+=("$a") ;;
    esac
    [[ "${a,,}" =~ ^use_camera:=(true|1|yes|on)$ ]] && USE_CAMERA=true
done

LOG="$(mktemp -t rover_perception_bench.XXXXXX)"
LAUNCH_PIDS=()
if [[ "$USE_CAMERA" == true ]]; then
    setsid ros2 launch rover_realsense rover_realsense.launch.py "${DRIVER_ARGS[@]}" >> "$LOG" 2>&1 &
    LAUNCH_PIDS+=($!)
fi
setsid ros2 launch rover_perception_bringup rover_perception.launch.py "${PERCEPTION_ARGS[@]}" \
    >> "$LOG" 2>&1 &
LAUNCH_PIDS+=($!)
stop() {
    for pid in "${LAUNCH_PIDS[@]}"; do
        kill -INT -- "-$pid" 2> /dev/null || kill -INT "$pid" 2> /dev/null || true
    done
    sleep 3
    for pid in "${LAUNCH_PIDS[@]}"; do kill -TERM -- "-$pid" 2> /dev/null || true; done
}
trap stop EXIT

echo "warming up ${WARMUP}s (launch log: $LOG) ..."
sleep "$WARMUP"

pid_of() { pgrep -f "[${1:0:1}]${1:1}" | head -1 || true; }
cpu_ticks() { awk '{print $14 + $15}' "/proc/$1/stat" 2> /dev/null || echo 0; }
machine_ticks() { awk '/^cpu /{t=0; for (i=2;i<=NF;i++) t+=$i; print t, $5}' /proc/stat; }

declare -A PID T0
for p in "${PATTERNS[@]}"; do
    pid="$(pid_of "$p")"
    if [[ -n "$pid" ]]; then PID[$p]="$pid"; T0[$p]="$(cpu_ticks "$pid")"; fi
done
read -r M_TOTAL0 M_IDLE0 < <(machine_ticks)
START="$(date +%s.%N)"

# Topic rates are measured during the same window.
declare -A HZFILE
for t in "${TOPICS[@]}"; do
    f="$(mktemp -t rover_perception_hz.XXXXXX)"
    HZFILE[$t]="$f"
    timeout "$((DURATION - 2))" ros2 topic hz "/${NS:+$NS/}$t" --window 30 > "$f" 2>&1 &
done
sleep "$DURATION"

END="$(date +%s.%N)"
read -r M_TOTAL1 M_IDLE1 < <(machine_ticks)
ELAPSED="$(awk -v a="$START" -v b="$END" 'BEGIN{print b-a}')"

echo
echo "== machine: $(lscpu | awk -F: '/Model name/{gsub(/^ +/,"",$2); print $2; exit}'), $(nproc) cores, $(free -m | awk 'NR==2{print $2}') MB RAM"
if command -v vcgencmd > /dev/null; then echo "   throttled: $(vcgencmd get_throttled)  temp: $(vcgencmd measure_temp)"; fi
echo "   args: ${ARGS[*]}   window: $(printf '%.0f' "$ELAPSED") s"
printf '%-26s %12s %10s\n' process "CPU (% of 1 core)" "RSS (MB)"
TOTAL=0
for p in "${PATTERNS[@]}"; do
    [[ -n "${PID[$p]:-}" ]] || continue
    t1="$(cpu_ticks "${PID[$p]}")"
    cpu="$(awk -v a="${T0[$p]}" -v b="$t1" -v hz="$TICKS" -v s="$ELAPSED" 'BEGIN{printf "%.1f", (b-a)/hz/s*100}')"
    rss="$(awk '/VmRSS/{printf "%.0f", $2/1024}' "/proc/${PID[$p]}/status" 2> /dev/null || echo 0)"
    printf '%-26s %12s %10s\n' "$p" "$cpu" "$rss"
    TOTAL="$(awk -v a="$TOTAL" -v b="$cpu" 'BEGIN{print a+b}')"
done
printf '%-26s %12.1f\n' "perception total" "$TOTAL"
awk -v t0="$M_TOTAL0" -v t1="$M_TOTAL1" -v i0="$M_IDLE0" -v i1="$M_IDLE1" \
    'BEGIN{printf "whole machine busy: %.1f %% (all cores, all stacks)\n", (1-(i1-i0)/(t1-t0))*100}'
echo
printf '%-34s %s\n' topic "average rate (Hz)"
for t in "${TOPICS[@]}"; do
    r="$(grep -E 'average rate' "${HZFILE[$t]}" 2> /dev/null | tail -1 | awk '{print $3}')"
    printf '%-34s %s\n' "$t" "${r:--}"
    rm -f "${HZFILE[$t]}"
done
