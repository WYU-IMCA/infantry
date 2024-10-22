# infantry
步兵自瞄
使用的是中南大学FYT战队的框架
<<<<<<< HEAD
使用时把所有文件夹克隆到src目录里


## 文件目录结构 📂
```shell
.
├── src  根目录
|   ├── rm_auto_aim          自瞄模块
|   │    ├──armor_detector   装甲板检测模块
|   │    ├──armor_solver     装甲板追踪模块
|   │    └──rm_auto_aim
|   ├── rm_auto_record       录像模块
|   ├── rm_bringup           
|   ├── rm_buff              能量机关模块
|   ├── rm_harware_driver    相机驱动模块及串口模块
|   ├── rm_interfaces        自定义话题类型
|   ├── rm_robot_description 机器人建模文件
|   ├── rm_rune              能量机关模块
|   └── rm_utils             工具模块(包含pnp、弹道解算等)
|
└── rv_start.sh        一键启动的脚本
