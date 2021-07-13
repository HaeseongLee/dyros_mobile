# dyros_mobile
# 설치 방법
1. git clone https://github.com/HaeseongLee/dyros_mobile.git
2. cd powered_caster_vehicle
3. mkdir build
4. cd build
5. cmake ..
6. make

# CoppelisaSim 사용 방법
1. roscore 실행
2. coppeliaSim 실행 후 fixed_mobile_1.ttt 파일을 로드(~/powered_caster_vehicle/scene)
3. rosrun powered_caster_vehicle powered_caster_vehicle
4. 'o'를 누르면 미리 지정된 위치와 방위로 이동
