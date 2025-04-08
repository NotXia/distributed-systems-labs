## Build 
```
# From ./ros2/
colcon build --symlink-install --packages-select consensus
```


## Execute
```
# From ./ros2/
source install/setup.bash
ros2 launch consensus launch.py
```