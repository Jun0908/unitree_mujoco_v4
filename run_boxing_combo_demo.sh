#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${G1_VENV:-$HOME/g1_venv}"
LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs/boxing_combo_demo}"
VISUALIZER_DELAY="${VISUALIZER_DELAY:-2}"

mkdir -p "$LOG_DIR"

PIDS=()
TAIL_PID=""

cleanup() {
  local status=$?
  [[ -n "$TAIL_PID" ]] && kill "$TAIL_PID" 2>/dev/null || true
  if ((${#PIDS[@]})); then
    echo
    echo "Stopping processes..."
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait "${PIDS[@]}" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

activate_python() {
  [[ -d "$VENV_DIR" ]] && source "$VENV_DIR/bin/activate" || true
}

start_process() {
  local name="$1" workdir="$2"
  shift 2
  local log="$LOG_DIR/$name.log"
  : > "$log"
  ( activate_python; cd "$workdir"; exec "$@" ) >"$log" 2>&1 &
  PIDS+=("$!")
  echo "Started $name  (pid $!  log $log)"
}

echo "=== G1 Boxing Combo Demo ==="
echo "Root : $ROOT_DIR"
echo "Logs : $LOG_DIR"
echo

start_process "visualizer"   "$ROOT_DIR/simulate_python"  python3 -u g1_seiken_visualizer.py
sleep "$VISUALIZER_DELAY"
start_process "boxing_combo" "$ROOT_DIR/example/python"   python3 -u boxing_combo_keyframe.py

echo
echo "Running. Press Ctrl+C to stop."
echo

tail -n +1 -F \
  "$LOG_DIR/visualizer.log" \
  "$LOG_DIR/boxing_combo.log" &
TAIL_PID=$!

set +e
wait -n "${PIDS[@]}"
set -e
echo
echo "A process exited — shutting down."
