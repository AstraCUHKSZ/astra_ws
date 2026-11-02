pushd non_ros_src/lerobot_new >/dev/null
source ./.venv/bin/activate
.venv/bin/lerobot-edit-dataset \
  --repo_id=cgluWxh/grab_diff_bottle_multitask_new \
  --root=/home/aha-robot/.cache/huggingface/lerobot/cgluWxh/grab_diff_bottle_multitask_new \
  --operation.type=delete_episodes \
  --operation.episode_indices='[79]' \
  --push_to_hub=false

popd >/dev/null