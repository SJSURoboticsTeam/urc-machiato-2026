#!/usr/bin/env bash
# Clean colcon artifacts and rebuild ROS 2 packages needed for autonomy + BT.
# Uses canonical --base-paths (workspace root has COLCON_IGNORE).
# See docs/development/ros2_python_environment.rst and docs/development/BUILD_AND_TEST.md.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# If you already ran `source install/setup.bash` in this shell, then delete install/, colcon still
# sees stale AMENT_PREFIX_PATH / CMAKE_PREFIX_PATH and prints "path ... doesn't exist". Strip those.
strip_paths_under() {
    local varname="$1"
    local root="$2"
    local val="${!varname:-}"
    [ -z "$val" ] && return 0
    local kept=()
    local p
    IFS=':' read -ra parts <<< "$val"
    for p in "${parts[@]}"; do
        [ -z "$p" ] && continue
        if [[ "$p" == "$root"/* || "$p" == "$root" ]]; then
            continue
        fi
        kept+=("$p")
    done
    if [ "${#kept[@]}" -eq 0 ]; then
        unset "$varname"
    else
        local IFS=':'
        export "$varname=${kept[*]}"
    fi
}

INSTALL_ROOT="$PROJECT_ROOT/install"
for _v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH COLCON_PREFIX_PATH PYTHONPATH; do
    strip_paths_under "$_v" "$INSTALL_ROOT"
done

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "Sourcing ROS2..."
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "Error: ROS2 not found. Install and source e.g. /opt/ros/jazzy/setup.bash"
        exit 1
    fi
fi

# Match system Python to ROS (avoid conda/venv producing install/.../python3.13 while Jazzy uses 3.12)
export PATH="/usr/bin:/bin:/usr/local/bin:$PATH"
unset VIRTUAL_ENV
export PYTHON_EXECUTABLE="$(command -v python3)"
echo "Using PYTHON_EXECUTABLE=$PYTHON_EXECUTABLE ($("$PYTHON_EXECUTABLE" --version))"
if [[ "$PYTHON_EXECUTABLE" != /usr/bin/python3* ]]; then
    echo "WARNING: python3 is not /usr/bin/python3 — conda/another env may rebuild bindings for the wrong version."
    echo "         Prefer: conda deactivate && hash -r && re-run this script."
fi

echo "Cleaning build, install, log..."
rm -rf build install log

# Remove again in case paths were re-exported by anything in the shell profile
for _v in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH COLCON_PREFIX_PATH PYTHONPATH; do
    strip_paths_under "$_v" "$INSTALL_ROOT"
done

echo "Building autonomy_interfaces, autonomy_core, autonomy_bt..."
colcon build \
    --symlink-install \
    --base-paths \
        shared/interfaces/autonomy_interfaces \
        services/autonomy/autonomy_core \
        services/autonomy/bt \
    --cmake-args "-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}"

echo "Done. Source the workspace: source install/setup.bash"
