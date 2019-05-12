2019 DGIST DPoom Project Introduction Page
===

2019 DGIST 소형자율주행로봇 플랫폼 구축 및 RGB-D 센서를 이용한 Full Automation 자율 주행 알고리즘 구축<br/>

---

>__2019년 말까지는 해당 소스코드는 비공개 상태 예정입니다.__

## 개발 소개 ![build_status](https://img.shields.io/badge/build-WIP-yellow.svg)
> "길을 찾는 귀여운 소형 자율 주행로봇" <br/>
>"__뎁스카메라__ 를 이용한 __독립형 완전자율__ 주행"

### Origin of name
디품(Dpoom)은 개발자 중 한명의 애묘인 품이의 이름을 따서 Digitalized 품 | ~~DGIST 품~~ 을 노려서 만들었습니다.

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
        - 실내 자율주행 로봇 SW 개발 ([TRL](https://itec.etri.re.kr/itec/sub01/sub01_07.do) : 1 ~ 3)
        - 실내 자유 주향 센서 통합 방법 연구
        
 - 핵심 기술 키워드
    - SLAM
    - Path Planning
    - HCI, facial expressions
    - MORP
    - Quriosity Engine <Region Attention Engine>
  
 - 개발 환경
    - _SOFTWARE_
      - Python 3.6
      - Python 2.7 on Robot Platform
      - Realsense SDK 2
      - Ubuntu 16.04 LTS
      - Tensorflow 1.8.0
    - _HARDWARE_
      - Platform Computing Unit : Lattepanda Alpha 864
      - Intel Realsense Camera D435i
      - Turtlebot3 waffle pi


### Depth Solution _MORP™_ - Milestone Over Rendered Path
#### [About MORP](https://github.com/shinkansan/2019-UGRP-DPoom/tree/master/MORP)
현재 MORP는 연구개발 목적상 비공개로 분류되어있습니다. 완료 후 오픈소스 공개 예정
### Mainstream robot operationg method - About easy™ series
#### [About easy™ Series](https://github.com/shinkansan/2019-UGRP-DPoom/tree/master/easygo)
easyGO, easyControl, esayVector, easyDrive 는 All-in-one 파이썬 기반 Turtlebot3 주행 보조 라이브러리입니다.


### About overnight develop (9to9)
We have very unique and seems counter-productive tradition on this squad. It is called "9to9" it's overnight develop since 9 pm to next morning 9 am, Although it's quiet negative affect on our life on next day. Such as wake up at 9pm instead. But after several 9to9, we had lots of  significant breakthrough on our research.

### Develop Schedule
  1. 19' 1Q (formal 9to9 start since March)
    __Week 1.__ Setup development environment.<br/>
    __Week 2.__ Ground Segmentation. <br/>
    __Week 3.__ Turtlebot3 Waffle pi Setup for testbed and change platform Computer to Lattepanda Alpha.    <br/>

### Acknowledgement
This research is funded by <br/> DGIST Undergraduate research program (grants number : ugrp-21)<br/> SAMSUNG Research IRP (grants number : )
