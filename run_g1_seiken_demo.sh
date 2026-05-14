#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${G1_VENV:-$HOME/g1_venv}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs/g1_seiken_demo}"
VISUALIZER_DELAY="${VISUALIZER_DELAY:-2}"
CONTROLLER_DELAY="${CONTROLLER_DELAY:-3}"

mkdir -p "$LOG_DIR"

PIDS=()
NAMES=()
TAIL_PID=""

cleanup() {
  local status=$?

  if [[ -n "$TAIL_PID" ]] && kill -0 "$TAIL_PID" 2>/dev/null; then
    kill "$TAIL_PID" 2>/dev/null || true
  fi

  if ((${#PIDS[@]})); then
    echo
    echo "Stopping G1 Seiken demo processes..."
    for pid in "${PIDS[@]}"; do
      kill "$pid" 2>/dev/null || true
    done
    wait "${PIDS[@]}" 2>/dev/null || true
  fi

  exit "$status"
}

trap cleanup EXIT INT TERM

activate_python() {
  if [[ -d "$VENV_DIR" ]]; then
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
    return
  fi

  echo "Warning: virtualenv not found at $VENV_DIR; using current Python environment."
  echo "Set G1_VENV=/path/to/venv if you want to use a different environment."
}

start_process() {
  local name="$1"
  local workdir="$2"
  shift 2
  local log_file="$LOG_DIR/$name.log"

  : > "$log_file"
  (
    activate_python
    cd "$workdir"
    echo "[$name] cwd=$PWD"
    echo "[$name] command=$*"
    exec "$@"
  ) >"$log_file" 2>&1 &

  PIDS+=("$!")
  NAMES+=("$name")
  echo "Started $name (pid $!, log $log_file)"
}

echo "Starting G1 Seiken demo from $ROOT_DIR"
echo "Logs: $LOG_DIR"
echo

start_process "mock_nautilus" "$ROOT_DIR" python3 -u example/python/mock_nautilus_server.py
sleep "$VISUALIZER_DELAY"

start_process "g1_vs_g1_visualizer" "$ROOT_DIR/simulate_python" python3 -u g1_vs_g1_seiken_visualizer.py
sleep "$CONTROLLER_DELAY"

start_process "seiken_g1" "$ROOT_DIR/example/python" python3 -u seiken_g1.py

echo
echo "All three processes are running. Press Ctrl+C to stop them together."
echo

tail -n +1 -F \
  "$LOG_DIR/mock_nautilus.log" \
  "$LOG_DIR/g1_vs_g1_visualizer.log" \
  "$LOG_DIR/seiken_g1.log" &
TAIL_PID=$!

set +e
wait -n "${PIDS[@]}"
set -e
echo
echo "One of the demo processes exited; shutting down the rest."
