# guardian_surveillance
**A ROS package containing a lightweight and intelligent surveillence system embedded on the Raspberry Pi 4 Model B and implemented in the ROS ecosystem.**

## Overview
Guardian Surveillance is an affordable and portable surveillance system that combines computer vision and IoT device communication to deliver reliable and punctual security to any location that the user desires. Image processing techniques are deployed on real-time camera footage from the Raspberry Pi Camera Module V2 to detect, recognize, and identify potential intruders; the user's smartphone is subsequently alerted through SMS text messaging, and an email containing captured images of the intruder(s) are also sent to the user.

## Prerequisites
### Software
- Ubiquity Robotics Raspberry Pi Image: https://downloads.ubiquityrobotics.com/pi.html
	- Ubuntu 16.04 (Xenial)
	- ROS Kinetic
	- OpenCV
- SMFP
- Twilio 
### Hardware
- Raspberry Pi 4 Model B
- Raspberry Pi Camera Module V2

## Installation
### Install from Git Repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/guardian_surveillance.git
cd .. 
sudo apt-get install -y
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make # catkin build guardian_surveillance (if using catkin_tools)
source devel/setup.bash
source ~/.bashrc # get email and text credentials
rospack profile
```

## Setup
1. Enter email address and phone number (to receive alerts) in `/config/user_info.yaml`.
2. Create Twilio account: https://www.twilio.com/. The phone number generated will be used to send SMS text message alerts to the user.
3. Set private email and phone (Twilio) credentials as environment variables (i.e. to ~/.bashrc):
```
export SYSTEM_EMAIL_ADDRESS = <fill> # email address that will send email alert to user along with attached image of intruder
export SYSTEM_EMAIL_PASSWORD = <fill> 
export SYSTEM_PHONE_NUMBER = <fill> # phone number from Twilio that will send SMS text message alert to user
export SYSTEM_ACCOUNT_SID = <fill> # account SID from Twilio
export SYSTEM_AUTH_TOKEN = <fill> # authentication token from Twilio
```

## Nodes
- `intruder_detecter_node`
- `email_alerter_node`
- `text_alerter_node`

## Parameters
- `/intruder_detecter_node/show_stream` (bool): Show image processed camera feed with intruder detection.

## Usage
### Example 
- `roslaunch guardian_surveillance surveillance_system.launch`

## Contact
- Author and Maintainer: Joey Yang
- Email: joeyyang.ai@gmail.com
- GitHub: https://github.com/joeyjyyang
- LinkedIn: https://www.linkedin.com/in/joey-yang

