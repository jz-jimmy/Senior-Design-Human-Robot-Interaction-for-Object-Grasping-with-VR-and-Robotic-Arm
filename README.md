# Senior-Design-Human-Robot-Interaction-for-Object-Grasping-with-VR-and-Robotic-Arm
## Project Overview
This repository contains code, data, and documentation for our senior design project (ECE445). 

In this project, we presented a VR-guided robotic claw system that enabled intuitive remote manipulation through a Unity-based digital twin and Meta Quest VR interface. This system aimed at assistive and industrial applications, which enabled users to grasp objects in real time.

## Key Features 
- Feature 1: Achieved remote (~20 meters range) control of UR3e Robotic Arm in real-time (latency < 33ms)
- Feature 2: Created Digital Twins in Local Unity Project with an interface for VR to control through panels
- Feature 3: 3D-Printed a gripper in PLA + TPU materials with pressure self-adaptation mechanics
- Feature 4: Designed and soldered circuits on PCB 12 board to enable motor control of serial grasping
- Feature 5: Used two orthogonally placed cameras for precise manual calibration of the target object

## Requirements
### Hardware
- Meta VR 3S 128G
- UR3e Robotic Arm
### Software 
- Unityhub version == 3.12.1
- Unity3D version == 2022.3.2f1
- SteamLink
### Supported on the following Operating Systems
- Universal Windows Platform, Android

## Installation
git clone https://github.com/jz-jimmy/Senior-Design-Human-Robot-Interaction-for-Object-Grasping-with-VR-and-Robotic-Arm.git

cd Senior-Design-Human-Robot-Interaction-for-Object-Grasping-with-VR-and-Robotic-Arm

## DEMO
<img width="1489" height="849" alt="image" src="https://github.com/user-attachments/assets/7a9e0b74-7692-4b37-9d74-dcb8f9e70dc1" />

![00553649222446e335e5e9656c66706](https://github.com/user-attachments/assets/8e49d4e5-465e-492d-a838-45e68fc26891)




## Workflow
<img width="1731" height="1126" alt="image" src="https://github.com/user-attachments/assets/1fd99da3-d747-46bb-9d8d-6c0c7fa25301" />

<img width="1366" height="844" alt="image" src="https://github.com/user-attachments/assets/c5cb7535-48c2-4b77-ab21-41e820d353fa" />


## Project Structure
### SubModule 1 Motor & PCB
- STM32F407:The core MCU of control flow, which receives and sends message to motor
- CAN Module:Designed with TJA1050 chip to transmit the RX & TX message to CAN_H & CAN_L differential level
- Pressure Sensor:With LM393 chip, it will output low level if pressure exceeds threshold
- 42 Stepper Motor Driver:The sub-MCU, integrating FOC algorithm to control the motor with position cycles

<img width="820" height="512" alt="image" src="https://github.com/user-attachments/assets/c95a10a7-a49a-47ba-b72f-17d6376e1017" />

### SubModule 2 Gripper 
- Connecting Flange: Arm Interface
- Base: Secures motor & linkage
- Linkage: Transmits power to jaw
- Flexible Jaws: End-effector for grasping
<img width="326" height="572" alt="image" src="https://github.com/user-attachments/assets/cf66e587-fadf-4524-a9ef-edf985b43e10" />

<img width="344" height="562" alt="image" src="https://github.com/user-attachments/assets/3073caa4-ecfc-43e5-a406-bffc8ac45550" />

### SubModule 3 Digital Twin
Req: At least 10 Hz update frequency for all critical inputs
<img width="783" height="562" alt="image" src="https://github.com/user-attachments/assets/59a9ec2b-c311-4db1-acb1-77e80e676d28" />

### SubModule 4 VR 
- Chose SteamLink for panel control
- Use router to increase bandwidth for lower latency
- Used IVcam to streamline phone camera to VR

## Contact
jz030724@gmail.com



## Citation (BibTeX)
```bibtex
@misc{JZ_VR_Robotic_Arm,
  author       = {Jiayu Zhou, Ziming Yan, Yuchen Yang, Jingxing Hu},
  title        = {Human Robot Interaction for Object Grasping with VR and Robotic Arm},
  year         = {2025},
  publisher    = {GitHub},
  journal      = {GitHub repository},
  howpublished = {\url{[https://github.com/yourusername/yourrepo](https://github.com/jz-jimmy/Senior-Design-Human-Robot-Interaction-for-Object-Grasping-with-VR-and-Robotic-Arm)}}
}
```

## License

This project is licensed under the [MIT License](LICENSE).

