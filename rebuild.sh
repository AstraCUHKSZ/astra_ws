source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
rm -rf build log install
export PATH=/usr/bin:/bin:$PATH
colcon build --symlink-install
