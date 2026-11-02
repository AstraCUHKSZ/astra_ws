#!/usr/bin/env bash

set -eo pipefail

export DISPLAY=${DISPLAY:-:1}

BRIDGE_HOST=${ASTRA_BRIDGE_HOST:-127.0.0.1}
BRIDGE_PORT=${ASTRA_BRIDGE_PORT:-8765}
ROS_PYTHON_BIN=${ASTRA_ROS_PYTHON_BIN:-/usr/bin/python3}
ROS_LOG_DIR=${ASTRA_ROS_LOG_DIR:-/tmp/astra_ros_logs}
DATASET_REPO_ID=${ASTRA_DATASET_REPO_ID:-cgluWxh/grab_diff_bottle_multitask_new}
DATASET_ROOT=${ASTRA_DATASET_ROOT:-/home/aha-robot/.cache/huggingface/lerobot/cgluWxh/grab_diff_bottle_multitask_new}

# 每项格式为“目标 episode 总数::语言任务描述”。可以直接修改这里，也可以在
# 命令行传入同样格式的参数；命令行参数存在时会覆盖下面的默认配置。
TASK_SPECS=(
  "20::Use the left arm to pick up the left bottle"
  "20::Use the left arm to pick up the right bottle"
  "20::Use the right arm to pick up the right bottle"
  "20::Use the right arm to pick up the left bottle"
)

if (( $# > 0 )); then
  TASK_SPECS=("$@")
fi

TASK_DESCRIPTIONS=()
TASK_EPISODES=()

for spec in "${TASK_SPECS[@]}"; do
  if [[ "$spec" != *"::"* ]]; then
    echo "Invalid task specification: $spec" >&2
    echo 'Expected format: EPISODES::TASK DESCRIPTION' >&2
    exit 2
  fi

  episodes=${spec%%::*}
  description=${spec#*::}

  if ! [[ "$episodes" =~ ^[1-9][0-9]*$ ]]; then
    echo "Episode count must be a positive integer: $episodes" >&2
    exit 2
  fi

  if [[ -z "$description" ]]; then
    echo "Task description cannot be empty." >&2
    exit 2
  fi

  TASK_EPISODES+=("$episodes")
  TASK_DESCRIPTIONS+=("$description")
done

echo "Target state for $DATASET_REPO_ID:"
for i in "${!TASK_DESCRIPTIONS[@]}"; do
  printf '  %d. %s (target: %s episodes)\n' \
    "$((i + 1))" "${TASK_DESCRIPTIONS[$i]}" "${TASK_EPISODES[$i]}"
done
echo

(
  unset PYTHONHOME VIRTUAL_ENV
  export ROS_LOG_DIR
  mkdir -p "$ROS_LOG_DIR"
  source install/setup.bash
  "$ROS_PYTHON_BIN" -c 'import sys; assert sys.version_info[:2] == (3, 10), sys.version; import rclpy'
  PYTHONPATH="$PWD/src/astra_controller:${PYTHONPATH:-}" \
    exec "$ROS_PYTHON_BIN" -m astra_controller.lerobot_bridge_server \
      --host "$BRIDGE_HOST" \
      --port "$BRIDGE_PORT"
) &

BRIDGE_PID=$!

cleanup_bridge() {
  kill "$BRIDGE_PID" 2>/dev/null || true
  sleep 0.5
  kill -9 "$BRIDGE_PID" 2>/dev/null || true
}

trap cleanup_bridge EXIT

BRIDGE_READY=false
for _ in $(seq 1 50); do
  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    wait "$BRIDGE_PID"
    exit 1
  fi

  if timeout 1 bash -c "</dev/tcp/$BRIDGE_HOST/$BRIDGE_PORT" 2>/dev/null; then
    BRIDGE_READY=true
    break
  fi

  sleep 0.1
done

if [[ "$BRIDGE_READY" != true ]]; then
  echo "Astra LeRobot bridge did not start on $BRIDGE_HOST:$BRIDGE_PORT" >&2
  exit 1
fi

pushd non_ros_src/lerobot_new >/dev/null
source ./.venv/bin/activate

RESUME_DATASET=false
if [[ -d "$DATASET_ROOT" ]]; then
  echo
  echo "Found existing dataset directory:"
  echo "  $DATASET_ROOT"
  echo

  if [[ ! -f "$DATASET_ROOT/meta/info.json" && ! -d "$DATASET_ROOT/meta" ]]; then
    echo "Warning: this directory does not clearly look like a LeRobot dataset."
    echo
  fi

  while true; do
    read -r -p "Resume previous recording, start from scratch, or quit? [r/n/q] " choice
    case "$choice" in
      r|R|resume|Resume)
        RESUME_DATASET=true
        break
        ;;
      n|N|new|New)
        echo "Deleting existing dataset and starting from scratch:"
        echo "  $DATASET_ROOT"
        rm -rf -- "$DATASET_ROOT"
        break
        ;;
      q|Q|quit|Quit)
        echo "Aborted."
        exit 0
        ;;
      *)
        echo "Please enter r, n, or q."
        ;;
    esac
  done
fi

if lerobot-record --help 2>&1 | grep -q -- "--control.resume"; then
  RESUME_OPTION=--control.resume=true
else
  RESUME_OPTION=--resume=true
fi

count_task_episodes() {
  local dataset_root=$1
  local task_description=$2

  # LeRobot v3 把 episode metadata 存为 parquet；最后一段同时兼容旧版 jsonl。
  python - "$dataset_root" "$task_description" <<'PY'
import json
import sys
from pathlib import Path

import pandas as pd

root = Path(sys.argv[1])
wanted = sys.argv[2]
count = 0

episode_files = sorted((root / "meta" / "episodes").glob("**/*.parquet"))
if episode_files:
    for path in episode_files:
        frame = pd.read_parquet(path, columns=["tasks"])
        for tasks in frame["tasks"]:
            if isinstance(tasks, str):
                tasks = [tasks]
            if wanted in tasks:
                count += 1
else:
    legacy_path = root / "meta" / "episodes.jsonl"
    if legacy_path.exists():
        with legacy_path.open(encoding="utf-8") as stream:
            for line in stream:
                episode = json.loads(line)
                tasks = episode.get("tasks", [])
                if isinstance(tasks, str):
                    tasks = [tasks]
                if wanted in tasks:
                    count += 1

print(count)
PY
}

for i in "${!TASK_DESCRIPTIONS[@]}"; do
  task=${TASK_DESCRIPTIONS[$i]}
  target_episodes=${TASK_EPISODES[$i]}
  existing_episodes=0
  resume_args=()

  if [[ "$RESUME_DATASET" == true ]]; then
    resume_args+=("$RESUME_OPTION")
    existing_episodes=$(count_task_episodes "$DATASET_ROOT" "$task")
  fi

  if (( existing_episodes >= target_episodes )); then
    echo
    printf 'Skipping task %d/%d: %s\n' "$((i + 1))" "${#TASK_DESCRIPTIONS[@]}" "$task"
    printf 'Already has %d episodes; target is %d.\n' "$existing_episodes" "$target_episodes"
    continue
  fi

  num_episodes=$((target_episodes - existing_episodes))

  echo
  echo "============================================================"
  printf 'Task %d/%d: %s\n' "$((i + 1))" "${#TASK_DESCRIPTIONS[@]}" "$task"
  printf 'Existing: %d, target: %d, recording now: %d episodes\n' \
    "$existing_episodes" "$target_episodes" "$num_episodes"
  echo "============================================================"

  lerobot-record \
    --robot.type=astra_remote \
    --robot.host="$BRIDGE_HOST" \
    --robot.port="$BRIDGE_PORT" \
    --robot.wait_for_reset_timeout_s=300 \
    --dataset.fps=30 \
    --dataset.single_task="$task" \
    --dataset.repo_id="$DATASET_REPO_ID" \
    --dataset.root="$DATASET_ROOT" \
    --dataset.tags='["astra", "multitask"]' \
    --dataset.episode_time_s=-1 \
    --dataset.reset_time_s=-1 \
    --dataset.num_episodes="$num_episodes" \
    --dataset.push_to_hub=false \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --external_action_recording=true \
    --display_data=false \
    --play_sounds=false \
    "${resume_args[@]}"

  # 第一项完成后，后面的任务都追加到刚才创建/恢复的同一 dataset。
  RESUME_DATASET=true
done

popd >/dev/null

echo
echo "All tasks finished. Dataset saved at:"
echo "  $DATASET_ROOT"
