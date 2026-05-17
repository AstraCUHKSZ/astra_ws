export DISPLAY=:1
rm -rf /home/aha-robot/.cache/huggingface/lerobot/cgluWxh/eval_put_bottle/
source install/setup.bash
pushd non_ros_src/lerobot_new
lerobot-record \
  --robot.type=astra_joint \
  --dataset.fps=30 \
  --dataset.single_task="Pick the bottle and put it down elsewhere" \
  --dataset.repo_id=cgluWxh/eval_put_bottle \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=-1 \
  --dataset.reset_time_s=-1 \
  --dataset.push_to_hub=false \
  --policy.path=../lerobot/outputs/train/act_move_bottle/pretrained_model/ \
  --display_data=false
