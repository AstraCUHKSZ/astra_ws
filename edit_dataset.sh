pushd non_ros_src/lerobot_new >/dev/null
source ./.venv/bin/activate
.venv/bin/lerobot-edit-dataset \
  --repo_id=cgluWxh/eval_put_bottle \
  --root=/home/aha-robot/.cache/huggingface/lerobot/cgluWxh/eval_put_bottle \
  --operation.type=delete_episodes \
  --operation.episode_indices='[18]' \
  --push_to_hub=false

popd >/dev/null