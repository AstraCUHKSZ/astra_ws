export DISPLAY=:1
rm -rf /home/aha-robot/.cache/huggingface/lerobot/cgluWxh/eval_put_bottle/
source install/setup.bash
pushd non_ros_src/lerobot
python lerobot/scripts/control_robot.py \
 --robot.type=astra_joint \
 --control.type=record \
 --control.fps=30 \
 --control.single_task="Pick the bottle and put it down elsewhere" \
 --control.repo_id=cgluWxh/eval_put_bottle \
 --control.num_episodes=10 \
 --control.warmup_time_s=0 \
 --control.episode_time_s=-1 \
 --control.reset_time_s=-1 \
 --control.push_to_hub=false \
 --control.policy.path=outputs/train/act_move_bottle/pretrained_model/ \
 --control.run_compute_stats=false \
 --control.display_cameras=false