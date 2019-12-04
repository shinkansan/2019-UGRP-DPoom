2019 DGIST DPoom Project
==============================
## 연구 초록
본 연구팀은 비싼 라이다를 대신 해서 RGB-D (D for depth) 카메라인 Intel Realsense 를 이용하여, 자율 주행 솔루션을 개발하고 있다. 단순히 라이다의 비용적 측면에서의 개선이 아닌 뎁스 카메라를 이용해서 효율적인 로봇 구동을 위한 기술 개발을 하고 있다.
우선, 자율주행 알고리즘을 테스트할 플랫폼을 구축했다. 뎁스카메라를 이용해 주행 가능한 영역을 실시간으로 검출하여 주행하는 MORP 알고리즘을 개발했으며, 라이다가 아닌 뎁스카메라의 disparity map을 이용한 SLAM을 개발하여 실시간 위치 측위가 가능하다. 만들어진 맵에서 가고자하는 목적지를 정하면 path planning 알고리즘을 통해 경로를 생성하고, 이를 자체 개발한 DAP 알고리즘을 통해 추종하며 주행한다. 앞서 설명한 모든 주행 알고리즘을 통합하여 본 연구팀이 구축한 플랫폼으로 테스트 중이다. 
본 연구의 최종 목표는 작고 귀엽게 만들어진 플랫폼이 DGIST 기숙사 로비에서 자율주행으로 돌아다니며 학생들과 상호작용하도록 만드는 것이다. 따라서, 주행 테스트가 완료된 후에는 학생들과 상호작용할 수 있도록 HCI 요소를 개발할 예정이다. 

For further information, please visit our [page](https://shinkansan.github.io/2019-UGRP-DPoom/)


### Original Dpoom Model
one of our team member's cat </br>
NAME : Poom[품, in korean]
![품이님 사진](./img/master.png)


#### Build Status Badge by [shield.io](https://shields.io/category/build)
