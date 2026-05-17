export DISPLAY=:1
BRIDGE_HOST=${ASTRA_BRIDGE_HOST:-127.0.0.1}
BRIDGE_PORT=${ASTRA_BRIDGE_PORT:-8765}

(
  source install/setup.bash
  PYTHONPATH="$PWD/src/astra_controller:${PYTHONPATH:-}" \
    exec python3 -m astra_controller.lerobot_bridge_server --host "$BRIDGE_HOST" --port "$BRIDGE_PORT"
) &
BRIDGE_PID=$!
trap 'kill "$BRIDGE_PID" 2>/dev/null || true' EXIT
sleep 2

pushd non_ros_src/lerobot_new
lerobot-record \
  --robot.type=astra_remote \
  --robot.host="$BRIDGE_HOST" \
  --robot.port="$BRIDGE_PORT" \
  --dataset.fps=30 \
  --dataset.single_task="Pick the bottle and put it down elsewhere" \
  --dataset.repo_id=cgluWxh/put_bottle \
  --dataset.tags='["astra"]' \
  --dataset.episode_time_s=-1 \
  --dataset.reset_time_s=-1 \
  --dataset.num_episodes=50 \
  --dataset.push_to_hub=false \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --display_data=false
