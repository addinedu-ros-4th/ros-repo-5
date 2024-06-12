# ros-repo-5
파이널 프로젝트 5조 저장소. 자율적 위기 대응 및 상호작용 안내 서비스 로봇

# Admin 
## To get started


```bash
git clone https://github.com/addinedu-ros-4th/ros-repo-5.git
```
## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros-repo-5/pam_admin_ws
colcon build
```
## Sourcing

```bash
source /opt/ros/humble/setup.bash
source source install/setup.bash
```
## Running the Code
---
### Terminal 1
```bash
ros2 run camera_pkg yolo_test_node

```
Yolo Client 실행

### Terminal 2
```bash
ros2 run connection robot_command_server
```
Yolo Service Server 실행

### Terminal 3
```bash
ros2 run ui_pkg gui
```
PyQt GUI 실행

### Terminal 4
```bash
cd ../pinkbot/
source /opt/ros/humble/setup.bash
source source install/setup.bash
rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```
pinkbot rviz 실행

# Robot 
---
## To get started


```bash
git clone https://github.com/addinedu-ros-4th/ros-repo-5.git
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros-repo-5/pam_robot_ws
colcon build
```

## Sourcing

```bash
source /opt/ros/humble/setup.bash
source source install/setup.bash
sudo chmod 666 /dev/video0
```

## Running the Code
---
### Terminal 1
```bash
sudo chmod 666 /dev/video0
ros2 launch camera_pkg camera.launch.py

```
Camera 토픽 발행

### Terminal 2
```bash
ros2 run ros_package Robot_Topic
```
Admin GUI에서 구독할 Topic 발행 

### Terminal 3
```bash
cd pinkbot/
source /opt/ros/humble/setup.bash
source source install/setup.bash
ros2 launch minibot_bringup bringup_robot.launch.py
```

### Terminal 4
```bash
cd pinkbot/
source /opt/ros/humble/setup.bash
source source install/setup.bash
ros2 launch minibot_navigation2 bringup_launch.py map:=/home/uesr_name/map.yaml
```
### Terminal 5
```bash
cd ~/ros-repo-5/pam_robot_ws
source /opt/ros/humble/setup.bash
source source install/setup.bash
ros2 run drive_pkg navigation_to_goal
```
지정된 WayPoint 주행
