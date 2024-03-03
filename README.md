# ackermann_vehicle

tested in ubuntu 20.04, ROS noetic
## 1. installation

```bash
cd catkin_ws/src
git clone https://github.com/0-keun/ackermann_vehicle.git
sudo apt install ros-noetic-ackermann-msgs
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## 2. run in gazebo
- activate model and map
```bash
roslaunch ackermann_vehicle_gazebo ackermann_vehicle_noetic.launch
```
- line following algorithm
```bash
cd catkin_ws/src/ackermann_vehicle/ackermann_vehicle_gazebo/scripts
python3 lane_detect.py
```

publish /cmd_vel -> 동작

테스트 해보고 싶은 경우
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.2}}'
```
## 3. 수정

- 맵 수정 시
	arkermann_vehicle_gazebo/worlds 에 world 파일 추가 후, ackermann_vehicle_noetic.launch에서 world 파일 선언된 부분 수정

---
## 주의사항
- lane_detect.py 사용시
  노란 선 겹칠 경우 오작동
  곡률이 너무 클 경우 카메라 앵글 벗어나는 문제 존재
