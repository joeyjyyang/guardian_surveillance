# guardian_surveillance
**A lightweight and intelligent surveillence system embedded on the Raspberry Pi and implemented in the ROS ecosystem.**

## Overview
Guardian Surveillance is an affordable and portable surveillance system that combines computer vision and IoT device communication to deliver reliable and punctual security to any location that the user desires. Image processing techniques are deployed on real-time camera footage from the Raspberry Pi Camera Module V2 to detect, recognize, and identify potential intruders; the user's smartphone is subsequently alerted through SMS text messaging, and an email containing captured images of the intruder(s) are also sent to the user.

## Prerequisites
- Ubuntu 16.04 (Xenial)
- ROS Kinetic
- raspicam node
- OpenCV
- SMFP
- Twilio 

## Installation
### Install from Git Repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/guardian_surveillance.git
cd .. 
catkin_make # catkin build guardian_surveillance (if using catkin_tools)
source devel/setup.bash
source ~/.bashrc # get email and text credentials
rospack profile
```

## Nodes
- `intruder_detecter_node`
- `email_alerter_node`
- `text_alerter_node`

## Parameters
- `/intruder_detecter_node/show_stream` (bool): Show image processed camera feed with intruder detection.

## Example Usage
`roslaunch guardian_surveillance surveillance_system.launch


