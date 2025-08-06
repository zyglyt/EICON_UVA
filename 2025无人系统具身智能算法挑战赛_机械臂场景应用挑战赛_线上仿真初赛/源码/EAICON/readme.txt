
运行机械臂仿真
cd /home/q/EAICON
sh run_jaka_sim.sh

ros1消息
/Jaka/get_end_effector_pose 获取末端位置和方向
/Jaka/set_end_effector_pose 设置末端位置和方向
/Jaka/get_end_effector_pose 获取夹爪位置值
/Jaka/set_end_effector_pose 设置夹爪位置值
/Jaka/camera_depth          获取深度相机
/Jaka/camera_info           获取相机信息
/Jaka/camera_rgb            获取RGB相机
/Jaka/get_jointstate        获取关节状态
/Jaka/tf                    
/Jaka/set_gripper_value     设置夹爪关节角度 范围0 - 0.04
/Jaka/gripper_is_captured   夹爪是否捕获成功

设置夹爪关节角度话题 /Jaka/set_gripper_value 按照0.001 进行增幅当/Jaka/gripper_is_captured 话题 返回True时代表抓取成功 关节角度停止增幅

标定仿真
sh run_jaka_checkerboard_sim.sh
/Jaka/set_jointstate        设置关节状态


运行机器狗仿真
cd /home/q/EAICON
sh run_go2_sim.sh

ros1消息
/odom 里程计
/Go2/imu IMU
/Go2/is_moving 是否移动 机械臂和机器狗同时只能控制一个 根据当前字段判断
/Go2/gripper_is_captured 是否捕获
/Go2/get_end_effector_pose 获取末端位姿
/Go2/set_end_effector_pose 设置末端位姿
/Go2/set_gripper_value 设置家爪值 0-0.03 0为闭合 0.03为打开
/Go2/cmd_vel_x 设置前后速度 正数往前 负数往后
/Go2/cmd_vel_y 设置左右速度 正数往左 负数往右
/Go2/cmd_vel_yaw 设置朝向 +- 左右
/Go2/global_camera 全局相机
/Go2/front_camera 前置相机
/Go2/wrist_camera_rgb 手部相机
/Go2/wrist_camera_info 
/Go2/wrist_camera_depth 
/Go2/point_cloud 点云
/Go2/laser_scan 激光雷达数据



