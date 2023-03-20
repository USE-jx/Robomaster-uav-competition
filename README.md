# Robomaster-uav-competition
大疆无人飞行挑战赛，急速穿圈

## packgaes
1 plan_and_control : ROS节点包，调用规划器和控制器 

2 tracking_controller : 实现的几何控制器 

3 trajectory_generator: 闭式解法生成miniJerk轨迹

## 运行
`roslaunch plan_and_control trajectory_replan.launch` 带圆心识别和重规划
