2019 DGIST DPoom Project
==============================
For further information, please visit our [page](https://shinkansan.github.io/2019-UGRP-DPoom/)

### Single board computer Lattepanda Alpha based indoor mobile robot DPoom, with fully autonomous driving system using single RGB-D camera Intel Realsense D435i. 

> Keywards: Autonomy, SLAM, ROS, RGB-D, Low-end, Global path planning, Local path planning, path tracking, control, Human-Robot interaction

2019 DGIST 소형자율주행로봇 플랫폼 구축 및 RGB-D 센서를 이용한 Full Automation 자율 주행 알고리즘 구축<br/>

# Installation
### [Follow installation guide here](installation_guide)

# How to run
## SLAM
Mapping should be preceded before deploying robots. See [SLAM Page](SLAM/README.md).
## Global path planning
Using pcd map, the robot can plan global path using our FMM based modified A*. See [GPP Page](pathplanning/README.md).
## Motion planning
The robot can follow the generated path by motion planner. Our motion planner is using our real-time ground segmentation method named MORP. See [Motion Planning (MORP) Page](MORP/README.md).
## Full autonomous driving
RGB-D localization, global path planning and motion planning are integrated in one python script. Just run 
---

>__2019/12/5 We opened our github repos to public!!.__

## 개발 소개 ![build_status](https://img.shields.io/badge/build-WIP-yellow.svg)
> "길을 찾는 귀여운 소형 자율 주행로봇" <br/>
>"__뎁스카메라__ 를 이용한 __독립형 완전자율__ 주행"

Click below for introduction video.
<div align="center">
  <a href="https://www.youtube.com/watch?v=9KRy7pCXqaM&feature=youtu.be"><img src="https://img.youtube.com/vi/9KRy7pCXqaM/0.jpg" alt="Introduction Video"></a>
</div>

<p float="left">
  <img src="docs/gif/DPoom_temp.gif" alt="drawing" width="409"/>
  <img src="docs/gif/MORP_test.gif" alt="drawing" width="335"/>
</p>

### Origin of name
디품(Dpoom)은 개발자 중 한 명의 애묘인 품이의 이름을 따서 Digitalized 품 | ~~DGIST 품~~ 을 노려서 만들었습니다.

### 개발자 소개
김태경 / 디지스트 16학번 @ktk1501<br/>임승현 / 디지스트 16학번 @SeunghyunLim<br/>신관준 / 디지스트 18학번 @shinkansan<br/>심건희 / 디지스트 18학번 @jane79


### Importance of R&D
저희는 비싼 라이다를 대신 해서 RGB-D (D for depth) 카메라인 Intel Realsense 를 이용하여, 자율 주행 솔루션을 개발하고 있습니다.
단순히 라이다의 비용적 측면에서의 개선이 아닌 뎁스 카메라를 이용해서 효율적인 로봇 구동을 위한 기술 개발을 하고 있습니다.
<br/>
 - 연구 개발 기술의 세계적 수준 <br/>
    - [X] 개념 정립 단계
    - [ ] 생산 활용 단계
    - [ ] 기술의 안정화
    - [ ] 쇠퇴기
 - 앞으로의 전망
    - RGB-D 카메라를 이용한 SLAM 및 그 데이터를 이용한 자율 주행 기술의 기술적 우위 확보 가능
    - 라이다등 단가 상승 요인의 센서의 대체 및 비전 센서 통합으로, 저렴한 통합 센서 도입 가능
    - 실내 자율 주행 알고리즘에 대한 기술 선점 가능
    
 - 연구개발의 목표
    - 연구 개발의 최종 목표
        - 실내 자율 주행 로봇 플랫폼 구축
        - 실내 자율주행 로봇 SW 개발 ([TRL](https://itec.etri.re.kr/itec/sub01/sub01_07.do) : 3 ~ 5)
        - 실내 자율주행 센서 통합 방법 연구
        
 - 핵심 기술 키워드
    - SLAM
    - Path Planning
    - MORP
    - HCI, facial expressions
    - Quriosity Engine <Region Attention Engine>
  
 - 개발 환경
    - _SOFTWARE_
      - Ubuntu 16.04 LTS
      - ROS Kinetic
      - Python 3.6
      - Python 2.7 on Robot Platform
      - Realsense SDK 2
      - Tensorflow 1.8.0
      - OpenCV
    - _HARDWARE_
      - Platform Computing Unit : Lattepanda Alpha 864
      - Intel Realsense Camera D435i
      - Turtlebot3 waffle pi
      - Outer HW printed by 3D-Printer


### Depth Solution _MORP™_ - Milestone Over Rendered Path
#### [About MORP](MORP.md)
소개 페이지로 넘어갑니다. 현재 MORP는 연구개발 목적상 실제 코드를 비공개로 분류했습니다. 완료 후 오픈소스 공개 예정
### Mainstream robot operationg method - About easy™ series
#### [About easy™ Series](easySeries.md)
소개 페이지로 넘어갑니다. easyGO, easyControl, esayVector, easyDrive 는 All-in-one 파이썬 기반 Turtlebot3 주행 보조 라이브러리입니다.
### SLAM with Depth Camera
#### [About SLAM](SLAM.md)
소개 페이지로 넘어갑니다. 현재 진행 중인 SLAM 개발의 소스코드는 연구개발 목적상 실제 코드를 비공개로 분류했습니다. 완료 후 오픈소스 공개 예정
### Path Planning (based on A* algorithm)
#### [About path planning](Pathplanning.md)
소개 페이지로 넘어갑니다. A* 알고리즘을 기반으로 Fast Marching Method를 통한 distance cost를 추가해 장애물에서 충분한 거리를 둔 경로를 생성하도록 만들어진 Path Planning Algorithm입니다.

### About overnight develop (9to9)
We have very unique and seems counter-productive tradition on this squad. It is called "9to9" it's overnight develop since 9 pm to next morning 9 am, although it's quiet negative affect on our daily life on next day. Such as wake up at 9pm instead. But after several 9to9, we had lots of significant breakthrough on our research.

### Quarterly Development Achievement
  1. 19' 1Q (formal 9to9 start since March)<br/>
      1. 로봇 플랫폼 구축
      2. 플랫폼 컴퓨팅 유닛 구축
      3. 기본 주행을 위한 파이썬 기반 주행 라이브러리 개발
      4. 뎁스 카메라 구동 파이프 라인 및 뎁스 맵 응용 소프트웨어 개발

  2. 19' 2Q (formal 9to9 start since March)<br/>
      1. 뎁스 카메라를 이용한 주행가능영역 개발
      2. 뎁스 카메라를 이용한 Object Active Tracking 개발
      3. 주행 라이브러리 고도화 (자유 곡률 주행, 경로 기반 안정적인 주행, 실시간 경로 이탈 자동 복귀)
      4. 전력 효율 위한 OpenCR(모터 드라이버) - LattePanda(Computing Unit) 간의 전력 공유 및 USB 전원부 우회 공급부 구성
    
  3. 19' 3Q (formal 9to9 start since March)<br/>
      1. RGB-D 카메라를 이용한 실시간 실내 측위 및 맵핑
      2. Human Interactive 기능 삽입
      3. Graph-Base Motion/Path Planning 솔루셔 개발 - 주행 라이브러리 고도화 일환
      4. 표정 인터렉선 개발
      


#### Build Status Badge by [shield.io](https://shields.io/category/build)
