#!/usr/bin/env bash
# Drive the C++ and Python ROS 2 efficiency-test launches and capture
# `perf stat` for each node, then print a single summary table of CPU
# utilization (% of one CPU). Designed to run inside the companion
# Docker image, but works on any host with ROS 2 Humble + perf.

export LC_ALL=C

WS="${WS:-/ros2_ws}"
PERF_DURATION="${PERF_DURATION:-10}"
SPINUP_WAIT="${SPINUP_WAIT:-8}"
NODES=(perception_node camera_node imu_node planning_node control_node)

source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

if ! command -v perf >/dev/null 2>&1; then
    echo "[FATAL] 'perf' not found on PATH" >&2
    exit 1
fi

LAUNCH_PID=""
declare -A RESULTS    # key="${label}|${node}" -> "12.3%" or "n/a"

wait_for_exit() {
    local pid="$1" timeout="$2" i=0
    while [ "$i" -lt "$timeout" ] && kill -0 "$pid" 2>/dev/null; do
        sleep 1
        i=$((i + 1))
    done
}

stop_launch() {
    if [ -n "${LAUNCH_PID}" ] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "Stopping launch PID ${LAUNCH_PID} ..."

        kill -SIGINT -- -${LAUNCH_PID} 2>/dev/null || true

        for i in {1..10}; do
            if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
                break
            fi
            sleep 1
        done

        if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            echo "[WARN] launch did not exit, forcing kill"
            kill -SIGKILL -- -${LAUNCH_PID} 2>/dev/null || true
        fi

        wait "${LAUNCH_PID}" 2>/dev/null || true
    fi
    LAUNCH_PID=""
}

cleanup() {
    if [ -n "${LAUNCH_PID}" ]; then
        kill -SIGINT -- -${LAUNCH_PID} 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

find_pid() {
    local strategy="$1" pkg="$2" node="$3"
    if [ "${strategy}" = "pidof" ]; then
        pidof -s "${node}" 2>/dev/null || true
    else
        pgrep -f "${pkg}/lib/${pkg}/${node}" 2>/dev/null | head -n 1 \
            || pgrep -f "${node}"            2>/dev/null | head -n 1 \
            || true
    fi
}

# Parse "# 0.123 CPUs utilized" out of a perf-stat log → "12.3" (no %)
parse_cpu_pct() {
    local logfile="$1"
    awk '
        /CPUs utilized/ {
            for (i = 1; i <= NF; i++) {
                if ($i == "#") {
                    val = $(i+1); gsub(",", "", val)
                    printf "%.1f", val * 100
                    exit
                }
            }
        }
    ' "${logfile}"
}

run_test() {
    local label="$1" pkg="$2" launchfile="$3" pid_strategy="$4"

    echo
    echo "================================================================"
    echo "  ${label} efficiency test  (pkg=${pkg})"
    echo "================================================================"

    setsid ros2 launch "${pkg}" "${launchfile}" >"/tmp/${label}_launch.log" 2>&1 &
    LAUNCH_PID=$!
    echo "Launched ${pkg} ${launchfile}  (launch PID=${LAUNCH_PID})"
    echo "Waiting ${SPINUP_WAIT}s for nodes to come up ..."
    sleep "${SPINUP_WAIT}"

    local total=0 missing=0
    for node in "${NODES[@]}"; do
        echo
        echo "----- perf stat: ${label} / ${node} -----"
        local pid
        pid="$(find_pid "${pid_strategy}" "${pkg}" "${node}")"
        if [ -z "${pid}" ]; then
            echo "[WARN] could not locate PID for ${node}; skipping"
            RESULTS["${label}|${node}"]="n/a"
            missing=1
            continue
        fi
        echo "PID=${pid}"
        local logfile="/tmp/${label}_${node}.perf"
        perf stat -p "${pid}" -- sleep "${PERF_DURATION}" 2>"${logfile}" || true
        cat "${logfile}"
        local pct
        pct="$(parse_cpu_pct "${logfile}")"
        if [ -n "${pct}" ]; then
            RESULTS["${label}|${node}"]="${pct}%"
            total="$(awk -v t="${total}" -v v="${pct}" 'BEGIN{printf "%.1f", t + v}')"
        else
            RESULTS["${label}|${node}"]="n/a"
            missing=1
        fi
    done

    if [ "${missing}" -eq 0 ]; then
        RESULTS["${label}|all"]="${total}%"
    else
        RESULTS["${label}|all"]="${total}%*"   # * = partial (some nodes missing)
    fi

    echo
    echo "Stopping ${label} nodes ..."
    stop_launch "${pkg}"
    sleep 2
}

run_test "cpp" "ros2_efficiency_test"    "efficiency_test.launch.py"    "pidof"
run_test "py"  "ros2_efficiency_test_py" "efficiency_test_py.launch.py" "pgrep"

echo
echo "================================================================"
echo "  CPU utilization summary  (% of one CPU, ${PERF_DURATION}s window)"
echo "================================================================"
printf "%-8s  %-18s  %10s\n" "package" "node" "cpu"
printf "%-8s  %-18s  %10s\n" "--------" "------------------" "----------"
for label in cpp py; do
    for node in "${NODES[@]}" all; do
        printf "%-8s  %-18s  %10s\n" \
            "${label}" "${node}" "${RESULTS[${label}|${node}]:-n/a}"
    done
done
echo "(* = partial: at least one node PID could not be located)"
