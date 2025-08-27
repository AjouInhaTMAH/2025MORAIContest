# VirtualAutoRace2025_System
 2025 8월에 주최한 가상환경 기반 자율주행 경진대회에 대한 git입니다. 
## 📌 소개  
 저희는 자율주행 시뮬레이션 대회 환경에서 차량의 인지, 판단, 제어를 통합적으로 수행하기 위해 개발된 시스템입니다.  
ROS 기반으로 구성되었으며, 센서 데이터 처리 → 상황 판단 → 주행 제어까지 전체 파이프라인을 **FSM(유한 상태 기계, Finite State Machine)** 으로 구현하였습니다.  

## 🖥️ 시스템 설명
### 🔹 소프트웨어 구조도
<img width="1936" height="1151" alt="Image" src="https://github.com/user-attachments/assets/f0deb214-563d-40c4-bd29-8749927f464a" />

---

### 🔹 시스템 구조도
<img width="1271" height="705" alt="Image" src="https://github.com/user-attachments/assets/137cf063-5ba2-4eb4-a353-6e3dae0e7c96" />

---

### 🔹 파일 구조도
<img width="1171" height="528" alt="Image" src="https://github.com/user-attachments/assets/256f633c-b9ab-4a74-a4b3-06847d64583c" />

---

### 만든 ROS 패키지 명
- auto_drive_sim
- wego

---

### 📝 시스템에 대한 설명

시스템은 ROS의 통신 방식을 적극 활용하여 자율주행의 핵심 기능들을 모듈화하고 분리하여 설계하였습니다.  
센서로부터 입력되는 데이터를 **Perception(인지)** 모듈에서 처리하고, 이를 기반으로 **Decision(판단)** 모듈이 주행 전략을 결정합니다. 이 과정에서 두 모듈 간의 데이터 교환은 ROS의 **topic**을 통해 이루어지도록 구축하였습니다.  
한편, 실제 차량을 제어하는 **Controller(제어)** 모듈의 경우, ROS 통신을 별도로 두면 불필요한 오버헤드가 발생한다고 판단하여, Decision 모듈 내부에서 클래스를 직접 호출하여 활용하는 방식으로 구현하였습니다.  

---

시스템의 동작은 크게 **인지 → 판단 → 제어** 흐름으로 정리할 수 있습니다.  

- **인지(Perception)**: SLAM의 `amcl_pose` 값과 카메라, 라이다, 신호등 데이터를 활용하여 주행 환경을 인식합니다.  
- **판단(Decision)**: 인지된 정보를 기반으로 **FSM(유한 상태 기계, Finite State Machine)** 을 이용하여 상황에 맞는 상태 전환을 수행합니다. 또한 `amcl_pose`는 FSM이 예기치 못하게 무너질 경우를 대비한 보조 장치로 사용됩니다.  
- **제어(Control)**: 차량 제어는 두 가지 방식으로 구현하였습니다. 하나는 차선을 기반으로 주행을 수행하는 방식이고, 다른 하나는 특정 상황에서 일정한 제어 값을 직접 전달하는 방식입니다. 이를 통해 다양한 주행 시나리오에 대응할 수 있도록 설계하였습니다.

## 실험 영상
### 전체 실험 영상
- [시뮬레이션 영상](https://youtu.be/uGjRywzh-Iw)
- [알고리즘 영상](https://youtu.be/tRIODtBzdAg)
### 부분 실험 영상
- [미션 1 (SLAM, 정적, 동적 피하며 배달하기)](https://youtu.be/JdML5Lk6EzU)
- [미션 2 (차선 따라가며 정적 장애물 피하기)](https://youtu.be/Pq0KJhjP2Lc)
- [미션 3 (차선 따라가며 동적 장애물에 멈추기)](https://youtu.be/JxACHl2vfio)
- [미션 4 (회전 교차로에 부딪치지 않고 들어가고 나가기)](https://youtu.be/8PETpChfBTI)
- [미션 5 (신호등에 따라 교차로 지나가기)](https://youtu.be/qY2_BoWOfKk)
## ✅ 잘한 점
- 모듈화된 구조로 유지보수 용이(이벤트 브로커 방식을 모방하였기에 가능한 일)
- ROS 기반으로 확장성과 재사용성이 높음  
- 시뮬레이터와 실제 차량 코드의 호환성을 고려한 설계  
- 다양한 테스트 환경에서 안정적으로 동작 확인(컴퓨터 성능에 따라 조금 달라지지만 괜찮다.)

## ⚠️ 아쉬웠던 부분
- SLAM에서 동작할 때 동적인 부분을 캐치 못하던것
- 대회 당일 날, 랜덤한 배달 위치로 테스트를 해보지 못했던 것
- 컴퓨터 1대에서는 속도가 느려지면서 알고리즘이 제대로 동작하지 못하는 현상, 이러한 현상은 가끔 일어나는데 성능이 좋으면 안일어난다. 즉 매번 할때마다 하드코딩의 경우 상황이 달라질 수 있다.

### 🔧 개선 방향
- SLAM에서 동적을 처리하기 위해서 따로 라이다 값을 수정해서 넘겨주었거나, costmap을 잘 이용했다면 괜찮았을 수도 있다.
- 미션 1 환경을 미리 구축하고 실험했다면 동적으로 움직이는 물체에 부딛치는 상황을 볼 수 있었고, 이를 방지하도록 속도를 조절할 수 있었을 것이다.
- 컴퓨터를 2대인 환경에서 미리 설정을 하고 코드를 짜는게 좋다. 즉 미리미리 환경 구축하고, 대회에서 내가 한 값들을 빠르게 변화를 줄 수 있는 환경 구축이 핵심이었다.

---

### Requirements
```
rosdep install --from-paths . --ignore-src -r -y
```

### Installation
```
git clone git@github.com:AjouInhaTMAH/2025MORAIContest.git
catkin_make
```

### Run
```
source ./devel/setup.zsh
```
or
```
source ./devel/setup.bash
```
#### 마지막 모든 패키지 실행법
```
roslaunch auto_drive_sim main.launch
```
#### SLAM 제외 부분 실행법
만약 SLAM을 제외하고, amcl 부분을 제외하여 FSM을 사용하고 싶다면 
먼저 auto_drive_sim 패키지의 dec_main.py의 파일의 
```
174 ~ 176 line
        # self.mission_mode = 0  # 0=기본, 1=왼쪽 차선만, 2=오른쪽 차선만 등
        # self.kill_slim_mover()
        # self.start_flag = True
```
을 활성화 시켜줍니다.
그 다음 auto_drive_sim 패키지의 per.launch의 파일의 
```
  <node pkg="auto_drive_sim" type="per_nav.py" name="per_car_nav_node" output="screen" />
```
부분을 비활성화시켜줍니다.

위 두 부분은 SLAM부분 연동을 끊는 부분과 미션에 해당하는 위치 pub부분을 조절해준 것입니다.
이후
```
roslaunch auto_drive_sim per.launch
roslaunch auto_drive_sim dec.launch
```
로 실행해줍니다.

---

## 기술 스택
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white) ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![ROS](https://img.shields.io/badge/-ROS-22314E?style=for-the-badge&logo=ROS)

## 📜 라이센스
본 프로젝트는 **BSD 3-Clause License**를 따릅니다.  

---

## 📂 참고한 Git
- [ROS 공식 GitHub](https://github.com/ros)  
- 참고한 다른 대회 자료
- [2023MORAIContest](https://github.com/lovelyoverflow/2023MORAIContest)  
- [MORAI-Contest](https://github.com/lococaeco/MORAI-Contest?tab=readme-ov-file)
- 기타 SLAM과 관련된 부분은 licenses 폴더를 참고 바랍니다.

---

## 👥 팀원 소개
| 프로필 | 이름 | 역할 / 담당 |
|--------|------|--------------|
|  | [박재일](https://github.com/parkjaeil00) | 미션 1 (SLAM & Nav) |
|  | [이강태](https://github.com/kante2) | 미션 2 & 3 (동적, 정적 장애물 피하면서 주행) |
|  | [유태현](https://github.com/kevin3183) | 미션 2 & 3 (동적, 정적 장애물 피하면서 주행) |
| <img src="https://github.com/user-attachments/assets/413060e9-eeeb-4ed7-8270-e89f2c175d99" width="100" height="200"> | [이선우](https://github.com/malenwater) | 미션 4, 시스템 총괄 및 깃허브 관리 (로터리 주행) |
| | [하재민](https://github.com/hajaemingood) | 미션 5, 차선 주행 알고리즘 구축 (신호등, 차선 따라 주행) |
