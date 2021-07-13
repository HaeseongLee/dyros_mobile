# powered_caster_vehicle
# 설치 방법
```sh
cd ~/catkin_ws/src
git clone --recursive https://github.com/HaeseongLee/powered_caster_vehicle.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# CoppelisaSim 사용 방법
1. roscore 실행
2. coppeliaSim 실행 후 fixed_mobile_1.ttt 파일을 로드(~/catkin_ws/src/powered_caster_vehicle/scene)
3. rosrun powered_caster_vehicle powered_caster_vehicle
4. 'o'를 누르면 미리 지정된 위치와 방위로 이동
