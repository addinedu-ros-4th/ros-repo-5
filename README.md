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

### Terminal 2
```bash
ros2 run connection robot_command_server
```

### Terminal 3
```bash
ros2 run ui_pkg gui
```

### Terminal 4
```bash
cd ../pinkbot/
source /opt/ros/humble/setup.bash
source source install/setup.bash
rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/nav2_view.rviz
```


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

### Terminal 2
```bash
ros2 run ros_package Robot_Topic
```

### Terminal 3
```bash
cd pinkbot/
source /opt/ros/humble/setup.bash
source source install/setup.bash
ros2 launch minibot_bringup bringup_robot.launch.py
```

### Terminal 3
```bash
cd pinkbot/
source /opt/ros/humble/setup.bash
source source install/setup.bash
ros2 launch minibot_navigation2 bringup_launch.py map:=/home/uesr_name/map.yaml
```





