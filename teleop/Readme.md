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
python teleop/keyboard_control.py
# eef publisher
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