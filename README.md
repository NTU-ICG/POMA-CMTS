# Toward Collaborative Multi-Target Search and Navigation with Attention Enhanced Local Observation
[![MIT License](https://img.shields.io/badge/license-MIT-green)](https://opensource.org/licenses/MIT)

## 1. Introduction
This repository contains the program files used for the work "Toward Collaborative Multitarget Search and Navigation with Attention Enhanced Local Observation" in Adanced Intelligent Systems. If you are interested, it would be grateful to cite our work and give us a star 🌟.

*Abstract: Collaborative multitarget search and navigation (CMTSN) is highly demanded in complex missions such as rescue and warehouse management. Traditional centralized and decentralized approaches fall short in terms of scalability and adaptability to real-world complexities such as unknown targets and large-scale missions. This article addresses this challenging CMTSN problem in three-dimensional spaces, specifically for agents with local visual observation operating in obstacle-rich environments. To overcome these challenges, this work presents the POsthumous Mix-credit assignment with Attention (POMA) framework. POMA integrates adaptive curriculum learning and mixed individual-group credit assignments to efficiently balance individual and group contributions in a sparse reward environment. It also leverages an attention mechanism to manage variable local observations, enhancing the framework's scalability. Extensive simulations demonstrate that POMA outperforms a variety of baseline methods. Furthermore, the trained model is deployed over a physical visual drone swarm, demonstrating the effectiveness and generalization of our approach in real-world autonomous flight.*

```
@article{xiao2024toward,
  title={Toward Collaborative Multitarget Search and Navigation with Attention-Enhanced Local Observation},
  author={Xiao, Jiaping and Pisutsin, Phumrapee and Feroskhan, Mir},
  journal={Advanced Intelligent Systems},
  pages={2300761},
  year={2024},
  publisher={Wiley Online Library}
}
```
<div align="left">
      <a href="https://www.bilibili.com/video/BV1wS421K7uv/">
         <img src="https://github.com/user-attachments/assets/a887e678-4690-489f-b65e-c2903e40b52a" width="700">
      </a>
</div>

## 2. Organization
The repository contains the following files:
1. `README.md`
2. asserts/
3. control/
4. models/

## 3. Physical_experiment
The content program files have been tested with a system of devices using Ubuntu 18.04/20.04 running with Python3 on AMD architecture.

### 3.1. Program Run
One device manager is required while multiple drone controllers may be used (must match with the number of DJI Tello EDU drones). Ensure that a drone is connected to the drone controller via wireless and that each drone controller is also connected to the device manager on the same network. A device may serve as both a device manager and a drone controller.

```
cd control/
python3 status_controller.py # for device manager only
python3 drone_controller.py # for drone controller device only
```

Before running a program, modify `telloIDs` and `goalIDs` in the `Experimental Variables` section of `drone_control.py` as follows:
- `telloIDs`: add the list of all the drone IDs, in any order that starts with the drone ID that is controlled by the device
- `goalIDs`: add the list of all the target IDs, in any order

### 3.2. File Content
1. `control/drone_control.py`: a Python script file for controlling an individual drone
2. `control/status_controller.py`: a Python script file for controlling the status of the overall system (all drones and all targets)
3. `control/poma_helper.py`: a Python script file for helper functions used in this work
4. `control/ros_helper.py`: a Python script file for helper functions related to ROS used in this work
5. `control/tellopy_helper.py`: a Python script file for helper functions related to TelloPy library used in this work
6. `models/poma_model.onnx`: an ONNX model file used by drones for their intelligence decision making

### 3.3. Requirements
#### Software Requirement
1. ROS Noetic or Melodic
2. Python 3
3. PyAV library
4. vrpn_client_ros (ROS1) Package
5. ONNX Runtime
6. Torch
7. Torchvision
8. OpenCV
9. PIL Library
10. Numpy
11. Unity: 22021.3.11f1
13. ml-agents: 0.27.0
14. ml-agents-envs: 0.27.0
15. Communicator API: 1.5.0
16. PyTorch: 1.8.2+cu111

#### System Requirement
1. Ubuntu 18.04 or 20.04
2. (Highly recommended, not required) AMD architecture



















