import rospy
from guardian_surveillance.msg import Object

import smtplib
from email.message import EmailMessage

sender_email = "guardiansurveillance.ai@gmail.com"
sender_password = "t2vqh97u"
receiver_email = "joeyyang.ai@gmail.com"
starttls_port = 587
smtp_server = "smtp.gmail.com"

msg = EmailMessage()
msg["X-Priority"] = "1"
msg["From"] = sender_email
msg["To"] = receiver_email
msg["Subject"] = "Urgent!"
msg.set_content("ALERT: INTRUDER DETECTED BY GUARDIAN SURVEILLANCE")

server = smtplib.SMTP(smtp_server, starttls_port)
server.starttls()
server.login(sender_email, sender_password)
#server.sendmail(sender_email, receiver_email, msg.as_string())
server.quit()

def objectCb(object_msg):
	if (object_msg.classification "Human" == and object_msg.confidence == 0.99):
		rospy.loginfo("test")

def intruderAlerter():
    rospy.init_node("intruder_alerter_node", anonymous = True)
    rospy.Subscriber("/guardian_surveillance/object", Object, objectCb)
    rospy.spin()

if __name__ == '__main__':
    intruderAlerter()

