export DISPLAY=:1
source install/setup.bash
pushd non_ros_src/lerobot
python lerobot/scripts/control_robot.py \
  --robot.type=astra_joint \
  --control.type=record \
  --control.single_task="Pick the bottle and put it down elsewhere" \
  --control.fps=30 \
  --control.repo_id=cgluWxh/put_bottle_2 \
  --control.tags='["astra"]' \
  --control.warmup_time_s=0 \
  --control.episode_time_s=-1 \
  --control.reset_time_s=-1 \
  --control.num_episodes=50 \
  --control.push_to_hub=false \
  --control.resume=false \
  --control.local_files_only=true \
  --control.run_compute_stats=true 
#  --control.resume=true