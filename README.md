lane_detection_project
====
[![lane_detection](http://img.youtube.com/vi/faHEmd_3msI/0.jpg)](https://youtu.be/faHEmd_3msI)


차선인식 프로젝트
===
> **프로그래머스 K-Digital Training 자율주행 데브코스**  
> 1:10 스케일 모형 차(Xycar)를 활용한 차선인식 프로젝트 (Lane keeping project using 1:10 scale model car(Xycar))  
> 개발기간 (Development period): 2023.10.28 ~ 2023.11.17
</br>

## 팀 소개
| 팀원 | 팀원 | 멘토 |
|:------:|:------:|:---:|
| 김나혜 | 허동욱 |이치현|
|[@nahye03](https://github.com/nahye03)|[@dongwookheo](https://github.com/dongwookheo)|[@hyunny223](https://github.com/hyuny223)|
</br>

## 프로젝트 소개
![track](https://github.com/dongwookheo/lane_detection_project/assets/124948998/5bf6f9fd-c2fb-48ec-b703-914d3b91bf98)
- 카메라 센서로 차선을 인식하여, Xycar가 주어진 코스를 완주할 수 있도록 한다  
- 블록을 넘어뜨릴 경우, 차선을 이탈할 경우 감점
</br>

## 환경 설정
### Requirements
- OpenCV 4.5.5
- ROS melodic
- Ubuntu 18.04 LTS
### Installation
```
git clone https://github.com/dongwookheo/lane_detection_project.git
```
```
cd lane_detection_project/thirdparty/OpenCV
```
```
git clone https://github.com/opencv/opencv.git
```
```
cd opencv
```
```
git checkout 4.5.5
```
```
cd ../build
```
```
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install ../opencv
```
- You can check the number of cores through the `nproc` command.
```
make -j<core_num>
```
```
make install
```
</br>

## 주요 기능
### modules
#### Common.hpp
- 차선의 상태를 나타내는 구조체, 유지 보수를 위한 PREC data type 선언
#### ImageProcessor.hpp, ImageProcessor.cpp
- 차선 검출을 위한 이미지 전처리 담당 클래스 정의
#### KalmanFilter.hpp, KalmanFilter.cpp
- 차선 예측을 위한 칼만 필터 클래스 정의
#### LaneDetector.hpp, LaneDetector.cpp
- 차선 검출 담당 클래스 정의
#### LaneManager.hpp, LaneManager.cpp
- 차선 검출 시스템을 총괄하는 클래스 정의
#### PIDController.hpp, PIDController.cpp
- PID 제어기 클래스 정의
#### XycarController.hpp, XycarController.cpp
- Xycar 컨트롤러 클래스 정의
#### draw.hpp
- 디버깅을 위한 그리기 함수 정의

### Run system
- 차선 인식 주행
```
roslaunch lane_detection main_xycar.launch
```

### Test Xycar teleop
- i: 속도 Up, k: 속도 Down, j,l: 좌우 조향
- 추후 개선 사항: 현재는 i,j,k,l 키 입력 후 Enter를 입력해야 반영. Enter 입력 없이 주행할 수 있도록 개선 필요.
```
roslaunch lane_detection teleop_xycar.launch
```
</br>

## Stacks
### Environment
<img src="https://img.shields.io/badge/ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white"> <img src="https://img.shields.io/badge/visualstudiocode-007ACC?style=for-the-badge&logo=visualstudiocode&logoColor=white"> <img src="https://img.shields.io/badge/clion-000000?style=for-the-badge&logo=clion&logoColor=white">
<img src="https://img.shields.io/badge/git-F04032?style=for-the-badge&logo=git&logoColor=white"> <img src="https://img.shields.io/badge/github-181717?style=for-the-badge&logo=github&logoColor=white"> 

### Config
<img src="https://img.shields.io/badge/yaml-CB171E?style=for-the-badge&logo=yaml&logoColor=white">

### Development
<img src="https://img.shields.io/badge/cplusplus-00599C?style=for-the-badge&logo=cplusplus&logoColor=white"> <img src="https://img.shields.io/badge/ros-22314E?style=for-the-badge&logo=ros&logoColor=white"> 
<img src="https://img.shields.io/badge/opencv-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white">
<img src="https://img.shields.io/badge/cmake-064F8C?style=for-the-badge&logo=cmake&logoColor=white">

### Communication
<img src="https://img.shields.io/badge/slack-4A154B?style=for-the-badge&logo=slack&logoColor=white"> <img src="https://img.shields.io/badge/notion-000000?style=for-the-badge&logo=notion&logoColor=white">
<img src="https://img.shields.io/badge/jira-0052CC?style=for-the-badge&logo=jira&logoColor=white">

