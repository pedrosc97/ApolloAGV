#!/bin/bash
# ApolloAGV on the Mac: build, test and simulate. Everything ROS runs inside the Docker dev image.
#
#   bash scripts/dev.sh image          build (or refresh) the dev image; first time takes several minutes
#   bash scripts/dev.sh build          colcon build
#   bash scripts/dev.sh test           colcon test, with results
#   bash scripts/dev.sh sim            whole stack against the 2D simulator; view at ws://localhost:8765 in Foxglove
#   bash scripts/dev.sh shell          a shell in the container (ROS and the workspace already sourced)
#   bash scripts/dev.sh run -- <cmd>   run one command in the container, e.g. run -- ros2 topic list
#
#   bash scripts/dev.sh test-native    the pure-Python tests straight on the Mac (no Docker, no ROS), firmware protocol included
#   bash scripts/dev.sh gen            regenerate ring_protocol.h / .py / .js from comms/protocol.yaml
#   bash scripts/dev.sh run2d [args]   closed-loop 2D run straight on the Mac, e.g. run2d --plot follow.png
#   bash scripts/dev.sh ring-ui        ring console in Chrome (Web Serial to the Arduino, or a simulated ring)
#
#   bash scripts/dev.sh webots-server  on the Mac, leave running: lets the container start Webots.app
#   bash scripts/dev.sh webots-check   in a second terminal: stock TurtleBot demo, proves container <-> Webots works
set -uo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$(dirname "$REPO")"                                   # ~/Projects/Apollo
export APOLLO_WEBOTS_SHARED="$WS/.webots_shared"          # read by colcon/docker/compose.yaml
COMPOSE=(docker compose -f "$REPO/colcon/docker/compose.yaml")
CMD="${1:-help}"; shift || true

# Keep a copy of the output next to the other workspace logs (not for interactive commands).
LOG_DIR="$WS/scripts"
case "$CMD" in image|build|test|test-native|run2d|webots-check)
  [ -d "$LOG_DIR" ] && exec > >(tee "$LOG_DIR/dev_agv_$CMD.log") 2>&1 ;;
esac

need_docker() {
  command -v docker >/dev/null 2>&1 || { echo "FAILED: 'docker' not found. Install Docker Desktop (or OrbStack) and start it."; exit 1; }
  docker info >/dev/null 2>&1 || { echo "FAILED: Docker is installed but not running. Start it and retry."; exit 1; }
  mkdir -p "$APOLLO_WEBOTS_SHARED"
}
in_container() { "${COMPOSE[@]}" run --rm --service-ports dev "$@"; }

case "$CMD" in
  image)
    need_docker
    "${COMPOSE[@]}" build dev && echo "== image apollo-agv:dev ready" ;;
  build)
    need_docker
    in_container bash -c 'cd /apollo/ros2_ws && colcon build --symlink-install' ;;
  test)
    need_docker
    in_container bash -c 'cd /apollo/ros2_ws && colcon build --symlink-install >/dev/null && source install/setup.bash \
      && colcon test --event-handlers console_direct+ 2>&1 | grep -E "passed|failed|error|skipped|FAILED|ERROR"; colcon test-result --verbose | tail -25' ;;
  sim)
    need_docker
    echo "== Foxglove: Open connection -> ws://localhost:8765   (Ctrl-C here to stop)"
    in_container bash -c 'cd /apollo/ros2_ws && colcon build --symlink-install >/dev/null && source install/setup.bash \
      && ros2 launch apollo_agv_bringup sim2d.launch.py '"$*" ;;
  shell)
    need_docker
    in_container bash ;;
  run)
    need_docker
    [ "${1:-}" = "--" ] && shift
    in_container "$@" ;;
  gen)
    cd "$REPO" && python3 comms/gen_protocol.py "$@" ;;
  test-native)
    cd "$REPO" && python3 comms/gen_protocol.py --check && python3 -m pytest -q -p no:cacheprovider \
      || echo "NOTE: needs pytest on the Mac (python3 -m pip install pytest pyyaml). The description test also needs xacro and is skipped without it." ;;
  run2d)
    cd "$REPO" && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="colcon/ros2_ws/src/apollo_agv_follow:colcon/ros2_ws/src/apollo_agv_sim" python3 -m apollo_agv_sim.run2d "$@" ;;
  ring-ui)
    # Web Serial needs a secure context; localhost counts. Chrome or Edge only.
    URL="http://localhost:8770/ring_console.html"
    echo "== Ring console at $URL   (Ctrl-C here to stop)"
    ( sleep 1; open -a "Google Chrome" "$URL" 2>/dev/null || open "$URL" ) &
    cd "$REPO/arduino/tools" && exec python3 -m http.server 8770 --bind 127.0.0.1 ;;
  webots-server)
    export WEBOTS_HOME="${WEBOTS_HOME:-/Applications/Webots.app}"
    [ -d "$WEBOTS_HOME" ] || { echo "FAILED: $WEBOTS_HOME not found. Install Webots R2025a or newer from https://cyberbotics.com"; exit 1; }
    mkdir -p "$APOLLO_WEBOTS_SHARED" "$WS/.webots_server"
    SERVER="$WS/.webots_server/local_simulation_server.py"
    [ -f "$SERVER" ] || curl -fsSL -o "$SERVER" https://raw.githubusercontent.com/cyberbotics/webots-server/main/local_simulation_server.py \
      || { echo "FAILED: could not download local_simulation_server.py"; exit 1; }
    echo "== Webots launch server for the container (port 2000). Leave this running; Ctrl-C to stop."
    exec python3 "$SERVER" ;;
  webots-check)
    need_docker
    echo "== Expect Webots to open on the Mac with a TurtleBot3 in an apartment. Ctrl-C to stop."
    in_container bash -c 'ros2 launch webots_ros2_turtlebot robot_launch.py' ;;
  *)
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
