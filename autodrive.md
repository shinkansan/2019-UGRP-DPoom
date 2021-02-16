Integration (SLAM, path planning and motion planning)
===

## Autonomus Driving for DPoom
DPoom is an indoor robot self-driving around the DGIST dormitory lobby. For its autonomous driving, such modules are essentially required.

  - *[SLAM](SLAM/README.md)* : Firstly, it must use SLAM to scan around the indoor terrain for creating a map. During self-driving, it is used for localization.

  - *[path planning](pathplanning/README.md)* : It plans a efficient and safe path to the destination using SLAM created map.

  - *[DAP](easygo/readme.md)* : DPoom could drive along the provided path using DAP, which listening encoder and IMU (from OpenCR).
  
  - *[autodrive.py](autodrive.py)* integrates navigation modules such as GPP and motion planner (MORP).
  
  [integration.py](integration.py) integrates all required modules when autonomous driving. The gif below was recorded when we test the robot to autonomous driving in DGIST domitory lobby. 

  
<img src="/docs/gif/dpoom_integration_2.gif" alt="drawing" width="540"/>
