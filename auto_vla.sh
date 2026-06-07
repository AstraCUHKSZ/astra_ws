export DISPLAY=:1
BRIDGE_HOST=${ASTRA_BRIDGE_HOST:-127.0.0.1}
BRIDGE_PORT=${ASTRA_BRIDGE_PORT:-8765}
ROS_PYTHON_BIN=${ASTRA_ROS_PYTHON_BIN:-/usr/bin/python3}
ROS_LOG_DIR=${ASTRA_ROS_LOG_DIR:-/tmp/astra_ros_logs}
DATASET_REPO_ID=${ASTRA_DATASET_REPO_ID:-cgluWxh/eval_put_bottle}
DATASET_ROOT=${ASTRA_DATASET_ROOT:-/home/aha-robot/.cache/huggingface/lerobot/cgluWxh/eval_put_bottle}

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
if [ "$BRIDGE_READY" != true ]; then
  echo "Astra LeRobot bridge did not start on $BRIDGE_HOST:$BRIDGE_PORT" >&2
  kill "$BRIDGE_PID" 2>/dev/null || true
  exit 1
fi

pushd non_ros_src/lerobot_new >/dev/null
source ./.venv/bin/activate

RESUME_ARGS=()

dataset_exists=false
if [ -d "$DATASET_ROOT" ]; then
  dataset_exists=true
fi

looks_like_lerobot_dataset=false
if [ -f "$DATASET_ROOT/meta/info.json" ] || [ -d "$DATASET_ROOT/meta" ]; then
  looks_like_lerobot_dataset=true
fi

if [ "$dataset_exists" = true ]; then
  echo
  echo "Found existing dataset directory:"
  echo "  $DATASET_ROOT"
  echo

  if [ "$looks_like_lerobot_dataset" != true ]; then
    echo "Warning: directory exists, but it does not clearly look like a LeRobot dataset."
    echo "Expected something like:"
    echo "  $DATASET_ROOT/meta/info.json"
    echo
  fi

  while true; do
    read -r -p "Resume previous recording, start from scratch, or quit? [r/n/q] " choice

    case "$choice" in
      r|R|resume|Resume)
        echo "Resuming existing dataset."

        if lerobot-record --help 2>&1 | grep -q -- "--control.resume"; then
          RESUME_ARGS+=(--control.resume=true)
        else
          RESUME_ARGS+=(--resume=true)
        fi

        break
        ;;

      n|N|new|New)
        echo "Deleting existing dataset and starting from scratch:"
        echo "  $DATASET_ROOT"
        rm -rf "$DATASET_ROOT"
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

lerobot-record \
  --robot.type=astra_remote \
  --robot.host="$BRIDGE_HOST" \
  --robot.port="$BRIDGE_PORT" \
  --dataset.fps=30 \
  --dataset.single_task="Pick the bottle and put it down elsewhere" \
  --dataset.repo_id="$DATASET_REPO_ID" \
  --dataset.root="$DATASET_ROOT" \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=-1 \
  --dataset.reset_time_s=-1 \
  --dataset.push_to_hub=false \
  --policy.path=./outputs/train/smolvla_move_bottle_new/025000/pretrained_model/ \
  --display_data=false \
  --play_sounds=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --dataset.vcodec=auto \
  --profile_timing=true \
  --async_policy=true \
  --async_policy_queue_threshold=8 \
  --policy.use_amp=false \
  "${RESUME_ARGS[@]}"
#  --async_policy_debug=true

popd >/dev/null
