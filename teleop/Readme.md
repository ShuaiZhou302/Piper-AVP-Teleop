the code to do infernece/teleop here

cd cobot_magic/Piper_ros_private-ros-noetic/
bash can_config.sh  #非常重要，只要有开机或者关机的情况，就一定要重新初始化
source devel/setup.bash

## 2.2 启动2条从臂,启动前需要将机械臂断电重启!!!!!!!!!!!
关掉机械臂供电的插排，需要拔掉两条主臂的航空插头，插排上电

conda activate aloha
roslaunch piper start_ms_piper.launch mode:=1 auto_enable:=true

conda activate aloha
roslaunch astra_camera multi_camera.launch
# rqt_image_view

cd aloha-devel
推理
bash act/inference.sh
# keyboard run, able to check whether each motor is working or not
python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/joint_keyboard_control.py
# eef publisher
python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/eef_keyboard_control.py
## 推理时注意安全, 如果发现推理表现不正常, 请立即中断代码或者断开机械臂电源



# annotation

| 左臂维度 | 右臂维度 | 含义 |
| --- | --- | --- |
| 0 | 7 | 底座 z 轴旋转 |
| 1 | 8 | 底座 joint |
| 2 | 9 | 大臂 joint |
| 3 | 10 | 小臂（以臂为轴旋转） |
| 4 | 11 | 小臂 joint |
| 5 | 12 | 腕部（以臂为轴旋转） |
| 6 | 13 | 夹爪（0-0.1，关-开） |



# TODO

1. 设置3.10的环境 然后根据piperros的库实现双臂的pin command pos control
2. 检查原本urdf 能不能用  不能就试试新的  新的可以用
3  检查rpy pose的解算问题  
    1. 看一下joint6到底是啥
    2. rpy 到底是世界坐标系下的rpy 还是当前坐标系的rpy




python /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/teleop/frame_visualize.py --xyz 0.051315 -0.011833 0.172909 --rpy 2.927 1.299 3.011 --html /tmp/frame_right.html
