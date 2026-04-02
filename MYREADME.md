运行前需要，否则controller加载不起来
``` bash
LD_LIBRARY_PATH=$HOME/Worktree/SRU_project/sru-robot-deployment/b2w_sim/b2w_controllers/third_party/onnxruntime/lib64:$LD_LIBRARY_PATH
```

启动 go2_sim
``` bash
ros2 launch b2w_gazebo_ros2 go2_gazebo.launch.py enable_rviz:=true
```