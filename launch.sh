#!/bin/bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release  #本仓库包含的功能包过多，建议限制同时编译的线程数
# 运行
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py 
