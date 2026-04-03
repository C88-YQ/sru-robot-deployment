运行前需要，否则controller加载不起来
``` bash
source ~/ros2_onnx/bin/activate
LD_LIBRARY_PATH=$HOME/Worktree/SRU_project/sru-robot-deployment/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:$LD_LIBRARY_PATH
```

启动 go2_sim
``` bash
cd ~/Worktree/SRU_project/sru-robot-deployment
source install/setup.bash
ros2 launch b2w_gazebo_ros2 go2_gazebo.launch.py enable_rviz:=true
```

启动 导航
``` bash
source ~/ros2_onnx/bin/activate
cd ~/Worktree/SRU_project/sru-robot-deployment
ros2 launch rl_nav_controller go2_nav_controller.launch.py
```

发送导航命令
```
ros2 topic pub -r 10 /rsl_joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"


ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 25.0, y: 20.0, z: 0.0}}
}"
```

发送固定速度命令
```
ros2 topic pub -r 10 /path_manager/path_manager_ros/nav_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

policy格式转换
```
source ~/ros2_torch_export/bin/activate
cd /home/c88/Worktree/SRU_project/sru-robot-deployment
python3 rl_nav_controller/scripts/export_go2_nav_policy.py
```