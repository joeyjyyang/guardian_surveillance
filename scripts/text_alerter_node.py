'''
  Author: Joey Yang
  Email: joeyyang.ai@gmail.com
  Description: 
    ROS node that exposes the text_alerter ROS service, which sends the user 
    an SMS text message upon intruder detection using the Twilio API.
'''

#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse

import os
from collections import OrderedDict
from twilio.rest import Client

sender_phone_ = os.environ["SYSTEM_PHONE_NUMBER"]
receiver_phone_ = rospy.get_param("/user_info/user_phone_number")

account_sid_ = os.environ["SYSTEM_ACCOUNT_SID"]
auth_token_ = os.environ["SYSTEM_AUTH_TOKEN"]

client_ = Client(account_sid_, auth_token_)

def textAlertCb(request):
  client_.messages.create(
    to = receiver_phone_,
    from_ = sender_phone_,
    body = "ALERT: INTRUDER DETECTED BY GUARDIAN SURVEILLANCE"
  )
  return EmptyResponse()

def textAlertServer():
  rospy.init_node("text_alerter_node")
  text_alert_server = rospy.Service("text_alerter", Empty, textAlertCb)
  rospy.spin()

if __name__ == '__main__':
  textAlertServer()
