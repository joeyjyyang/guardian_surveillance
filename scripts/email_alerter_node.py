'''
  Author: Joey Yang
  Email: joeyyang.ai@gmail.com
  Description: 
    ROS node that exposes the email_alerter ROS service, which gets the latest 
    media (.jpg) file taken by the camera of the intruder, and sends an email
    with the photo attached to the user's gmail account via SMTP.
'''

#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse

import os
import glob

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

sender_email_ = os.environ["SYSTEM_EMAIL_ADDRESS"]
sender_password_ = os.environ["SYSTEM_EMAIL_PASSWORD"]
receiver_email_ = rospy.get_param("/user_info/user_email_address")
starttls_port_ = 587
smtp_server_ = "smtp.gmail.com"

msg_ = MIMEMultipart()
msg_["X-Priority"] = "1"
msg_["From"] = sender_email_
msg_["To"] = receiver_email_
msg_["Subject"] = "Urgent!"

text_ = MIMEText("Warning! Intruder(s) Detected!")
msg_.attach(text_)

def getImageFile():
  all_image_files = glob.glob("/home/ubuntu/catkin_ws/src/guardian_surveillance/media/*.jpg")
  image_file = max(all_image_files, key=os.path.getctime)   
  return image_file

def sendEmail(image):
  msg_.attach(image)
  server = smtplib.SMTP(smtp_server_, starttls_port_)
  server.starttls()
  server.login(sender_email_, sender_password_)
  server.sendmail(sender_email_, receiver_email_, msg_.as_string())
  server.quit()

def emailAlertCb(request):
  image_file = getImageFile()
  image = MIMEImage(open(image_file, "rb").read(), name=os.path.basename(image_file))
  sendEmail(image)
  log_msg = "Emailed image %s"%image_file
  rospy.loginfo(log_msg)
  return EmptyResponse()

def emailAlertServer():
  rospy.init_node("email_alerter_node")
  email_alert_server = rospy.Service("email_alerter", Empty, emailAlertCb)
  rospy.spin()

if __name__ == '__main__':
  emailAlertServer()
   
