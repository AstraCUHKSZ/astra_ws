#!/usr/bin/env bash
# 参数化的实机评测脚本（由 auto_vla.sh 改来）。一条命令跑一个评测条件。
#
# 用法:
#   ./eval.sh -n 条件名 -t "prompt" [选项]
#
# 必填:
#   -n NAME        条件名。数据集会存成 eval_<NAME>
#   -t PROMPT      默认 prompt（用了 --prompts 时它只作兜底）
# 选项:
#   -e N           录多少条 episode（默认 5）
#   -c DIR         checkpoint 的 pretrained_model 目录（默认 $ASTRA_CHECKPOINT 或下面的 DEFAULT_CHECKPOINT）
#   -s TEXT        场景说明，记进运行日志（例: "3 bottles standard"）
#   --schedule S   中途换 prompt: "帧号:prompt;帧号:prompt"，每条 episode 都按这个切
#   --prompts F    文本文件，每行一个 prompt；本次运行的第 i 条 episode 用第 (i mod 行数) 行开局
#   --hotkeys F    文本文件，每行一个 prompt；运行中按数字键 1-9 立即切到对应行
#   --flush        换 prompt 时丢掉队列里的旧动作（机器人会停一下等新 chunk）；默认不丢，平滑交接
#   --no-annotate  跑完不进入逐条标注
#
# 例:
#   ./eval.sh -n ll_std -t "Use the left arm to pick up the left bottle" -e 10 -s "3 bottles standard"
#   ./eval.sh -n switch_ll_lr -t "Use the left arm to pick up the left bottle" \
#       --schedule "90:Use the left arm to pick up the right bottle" -s "3 bottles standard"
#   ./eval.sh -n ood_prompts -t "Pick up the bottle" --prompts tools/prompts/ood.txt -e 12
#
# 每次运行追加一行到 eval_notes/runs.jsonl；换 prompt 的事件记在数据集旁边的 prompt_switch_log/ 里。
set -eo pipefail

export DISPLAY=${DISPLAY:-:1}
BRIDGE_HOST=${ASTRA_BRIDGE_HOST:-127.0.0.1}
BRIDGE_PORT=${ASTRA_BRIDGE_PORT:-8765}
ROS_PYTHON_BIN=${ASTRA_ROS_PYTHON_BIN:-/usr/bin/python3}
ROS_LOG_DIR=${ASTRA_ROS_LOG_DIR:-/tmp/astra_ros_logs}
DATASET_USER=${ASTRA_DATASET_USER:-cgluWxh}
DATASET_BASE=${ASTRA_DATASET_BASE:-/home/aha-robot/.cache/huggingface/lerobot/$DATASET_USER}
DEFAULT_CHECKPOINT=./outputs/train/diff_multi/030000/pretrained_model/
QUEUE_THRESHOLD=${ASTRA_QUEUE_THRESHOLD:-8}

NAME="" PROMPT="" EPISODES=5 CHECKPOINT=${ASTRA_CHECKPOINT:-$DEFAULT_CHECKPOINT} SCENE=""
SCHEDULE="" PROMPTS_FILE="" HOTKEYS_FILE="" FLUSH=false ANNOTATE=true

usage() { sed -n '2,27p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-2}"; }

while (( $# > 0 )); do
  case "$1" in
    -n) NAME=$2; shift 2 ;;
    -t) PROMPT=$2; shift 2 ;;
    -e) EPISODES=$2; shift 2 ;;
    -c) CHECKPOINT=$2; shift 2 ;;
    -s) SCENE=$2; shift 2 ;;
    --schedule) SCHEDULE=$2; shift 2 ;;
    --prompts) PROMPTS_FILE=$2; shift 2 ;;
    --hotkeys) HOTKEYS_FILE=$2; shift 2 ;;
    --flush) FLUSH=true; shift ;;
    --no-annotate) ANNOTATE=false; shift ;;
    -h|--help) usage 0 ;;
    *) echo "Unknown argument: $1" >&2; usage ;;
  esac
done

[[ -n "$NAME" ]] || { echo "Missing -n NAME" >&2; usage; }
[[ "$NAME" =~ ^[A-Za-z0-9_]+$ ]] || { echo "NAME may only contain letters, digits and _" >&2; exit 2; }
[[ -n "$PROMPT" || -n "$PROMPTS_FILE" ]] || { echo "Missing -t PROMPT (or --prompts FILE)" >&2; usage; }
[[ "$EPISODES" =~ ^[1-9][0-9]*$ ]] || { echo "-e must be a positive integer" >&2; exit 2; }

ROOT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$ROOT_DIR"
abs() { [[ -z "$1" ]] && return 0; [[ "$1" = /* ]] && echo "$1" || echo "$ROOT_DIR/$1"; }
PROMPTS_FILE=$(abs "$PROMPTS_FILE"); HOTKEYS_FILE=$(abs "$HOTKEYS_FILE")
for f in "$PROMPTS_FILE" "$HOTKEYS_FILE"; do
  [[ -z "$f" || -f "$f" ]] || { echo "File not found: $f" >&2; exit 2; }
done
if [[ -z "$PROMPT" ]]; then PROMPT=$(grep -v '^\s*\(#\|$\)' "$PROMPTS_FILE" | head -1); fi

DATASET_REPO_ID="$DATASET_USER/eval_$NAME"
DATASET_ROOT="$DATASET_BASE/eval_$NAME"
NOTES_DIR="$ROOT_DIR/eval_notes"
mkdir -p "$NOTES_DIR"

echo "Condition : $NAME"
echo "Prompt    : $PROMPT"
[[ -n "$PROMPTS_FILE" ]] && echo "Per-episode prompts: $PROMPTS_FILE"
[[ -n "$SCHEDULE" ]] && echo "Schedule  : $SCHEDULE"
[[ -n "$HOTKEYS_FILE" ]] && echo "Hotkeys   : $HOTKEYS_FILE (digit keys 1-9)"
echo "Episodes  : $EPISODES"
echo "Checkpoint: $CHECKPOINT"
echo "Dataset   : $DATASET_ROOT"
echo

(
  unset PYTHONHOME VIRTUAL_ENV
  export ROS_LOG_DIR
  mkdir -p "$ROS_LOG_DIR"
  source install/setup.bash
  "$ROS_PYTHON_BIN" -c 'import sys; assert sys.version_info[:2] == (3, 10), sys.version; import rclpy'
  PYTHONPATH="$PWD/src/astra_controller:${PYTHONPATH:-}" \
    exec "$ROS_PYTHON_BIN" -m astra_controller.lerobot_bridge_server --host "$BRIDGE_HOST" --port "$BRIDGE_PORT"
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

RESUME_ARGS=()
EPISODES_BEFORE=0
if [[ -d "$DATASET_ROOT" ]]; then
  echo "Found existing dataset directory: $DATASET_ROOT"
  while true; do
    read -r -p "Resume previous recording, start from scratch, or quit? [r/n/q] " choice
    case "$choice" in
      r|R)
        if lerobot-record --help 2>&1 | grep -q -- "--control.resume"; then
          RESUME_ARGS+=(--control.resume=true)
        else
          RESUME_ARGS+=(--resume=true)
        fi
        EPISODES_BEFORE=$(python -c "import json,sys; print(json.load(open(sys.argv[1]))['total_episodes'])" \
          "$DATASET_ROOT/meta/info.json" 2>/dev/null || echo 0)
        break ;;
      n|N)
        echo "Deleting existing dataset and starting from scratch: $DATASET_ROOT"
        rm -rf -- "$DATASET_ROOT"
        break ;;
      q|Q) echo "Aborted."; exit 0 ;;
      *) echo "Please enter r, n, or q." ;;
    esac
  done
fi

PROMPT_ARGS=()
[[ -n "$SCHEDULE" ]] && PROMPT_ARGS+=(--prompt_schedule="$SCHEDULE")
[[ -n "$PROMPTS_FILE" ]] && PROMPT_ARGS+=(--prompt_per_episode="$PROMPTS_FILE")
[[ -n "$HOTKEYS_FILE" ]] && PROMPT_ARGS+=(--prompt_hotkeys="$HOTKEYS_FILE")
[[ "$FLUSH" == true ]] && PROMPT_ARGS+=(--prompt_switch_flush=true)

# 运行日志：先记下这次打算做什么，跑完再补一行实际录了多少条
python - "$NOTES_DIR/runs.jsonl" "$NAME" "$PROMPT" "$EPISODES" "$CHECKPOINT" "$SCENE" "$SCHEDULE" \
         "$PROMPTS_FILE" "$HOTKEYS_FILE" "$FLUSH" "$DATASET_ROOT" "$EPISODES_BEFORE" <<'PY'
import json, sys, time
keys = ["name", "prompt", "episodes_requested", "checkpoint", "scene", "schedule", "prompts_file",
        "hotkeys_file", "flush", "dataset_root", "episodes_before"]
rec = dict(zip(keys, sys.argv[2:]))
rec.update(event="start", time=time.strftime("%Y-%m-%d %H:%M:%S"))
open(sys.argv[1], "a").write(json.dumps(rec, ensure_ascii=False) + "\n")
PY

set +e
lerobot-record \
  --robot.type=astra_remote \
  --robot.host="$BRIDGE_HOST" \
  --robot.port="$BRIDGE_PORT" \
  --dataset.fps=30 \
  --dataset.single_task="$PROMPT" \
  --dataset.repo_id="$DATASET_REPO_ID" \
  --dataset.root="$DATASET_ROOT" \
  --dataset.num_episodes="$EPISODES" \
  --dataset.episode_time_s=-1 \
  --dataset.reset_time_s=-1 \
  --dataset.push_to_hub=false \
  --policy.path="$CHECKPOINT" \
  --display_data=false \
  --play_sounds=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --dataset.vcodec=auto \
  --profile_timing=true \
  --async_policy=true \
  --async_policy_queue_threshold="$QUEUE_THRESHOLD" \
  --policy.use_amp=false \
  "${PROMPT_ARGS[@]}" \
  "${RESUME_ARGS[@]}"
RECORD_STATUS=$?
set -e

EPISODES_AFTER=$(python -c "import json,sys; print(json.load(open(sys.argv[1]))['total_episodes'])" \
  "$DATASET_ROOT/meta/info.json" 2>/dev/null || echo "$EPISODES_BEFORE")
python - "$NOTES_DIR/runs.jsonl" "$NAME" "$EPISODES_BEFORE" "$EPISODES_AFTER" "$RECORD_STATUS" <<'PY'
import json, sys, time
rec = {"event": "end", "time": time.strftime("%Y-%m-%d %H:%M:%S"), "name": sys.argv[2],
       "episodes_before": int(sys.argv[3]), "episodes_after": int(sys.argv[4]), "exit_status": int(sys.argv[5])}
open(sys.argv[1], "a").write(json.dumps(rec, ensure_ascii=False) + "\n")
PY
echo
echo "Recorded episodes $EPISODES_BEFORE..$((EPISODES_AFTER - 1)) of eval_$NAME (lerobot-record exit status $RECORD_STATUS)"

if [[ "$ANNOTATE" == true && "$EPISODES_AFTER" -gt "$EPISODES_BEFORE" ]]; then
  python "$ROOT_DIR/tools/annotate_eval.py" "$DATASET_ROOT" --notes-dir "$NOTES_DIR" --scene "$SCENE" || true
fi

popd >/dev/null
exit "$RECORD_STATUS"
