#!/usr/bin/env bash
# Skeletal HIL: verify ROS 2 topics exist and optionally meet a minimum average rate.
#
# Usage:
#   source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
#   ./scripts/hardware/sensor_topic_smoke.sh [options] /topic1 [/topic2 ...]
#
# Or: URC_SMOKE_TOPICS="/a /b" ./scripts/hardware/sensor_topic_smoke.sh
#
# Environment:
#   URC_SMOKE_MIN_HZ   minimum average rate (default 0.1); set 0 to skip hz check
#   URC_SMOKE_HZ_SEC   duration for ros2 topic hz per topic (default 4)
#   URC_SMOKE_TOPICS   space-separated topics if none passed on CLI

set -euo pipefail

usage() {
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
}

MIN_HZ="${URC_SMOKE_MIN_HZ:-0.1}"
DURATION="${URC_SMOKE_HZ_SEC:-4}"
ECHO_ONCE=false
topics=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --min-hz)
            MIN_HZ="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --echo-once)
            ECHO_ONCE=true
            shift
            ;;
        -h | --help)
            usage
            ;;
        *)
            topics+=("$1")
            shift
            ;;
    esac
done

if [[ ${#topics[@]} -eq 0 && -n "${URC_SMOKE_TOPICS:-}" ]]; then
    # shellcheck disable=SC2206
    topics=($URC_SMOKE_TOPICS)
fi

if [[ ${#topics[@]} -eq 0 ]]; then
    echo "ERROR: no topics. Pass topic names or set URC_SMOKE_TOPICS." >&2
    exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 not found in PATH (source ROS and workspace setup.bash)." >&2
    exit 1
fi

normalize_topic() {
    local t="$1"
    if [[ "${t:0:1}" == / ]]; then
        echo "$t"
    else
        echo "/$t"
    fi
}

topic_listed() {
    local want="$1"
    ros2 topic list | grep -Fxq "$want"
}

fail=0

for t in "${topics[@]}"; do
    n=$(normalize_topic "$t")
    if ! topic_listed "$n"; then
        echo "FAIL: topic not in ros2 topic list: $n" >&2
        fail=1
        continue
    fi
    echo "OK: listed $n"

    if [[ "$ECHO_ONCE" == true ]]; then
        if ! ros2 topic echo "$n" --once >/dev/null 2>&1; then
            echo "FAIL: ros2 topic echo --once failed or timed out: $n" >&2
            fail=1
            continue
        fi
        echo "OK: echo --once $n"
    fi

    # Skip hz when MIN_HZ is 0 or negative
    if awk -v m="$MIN_HZ" 'BEGIN { exit !(m > 0) }'; then
        # shellcheck disable=SC2086
        output=$(timeout --signal=INT "${DURATION}s" ros2 topic hz "$n" 2>&1 || true)
        rate=$(echo "$output" | grep -E '^average rate:' | tail -1 | awk '{print $3}')
        if [[ -z "${rate:-}" ]]; then
            echo "FAIL: no average rate for $n within ${DURATION}s (no messages?)" >&2
            fail=1
            continue
        fi
        if awk -v r="$rate" -v m="$MIN_HZ" 'BEGIN { exit !(r + 0 >= m + 0) }'; then
            echo "OK: $n average rate ${rate} Hz (min ${MIN_HZ})"
        else
            echo "FAIL: $n average rate ${rate} Hz below min ${MIN_HZ}" >&2
            fail=1
        fi
    else
        echo "SKIP: hz check for $n (URC_SMOKE_MIN_HZ=$MIN_HZ)"
    fi
done

exit "$fail"
