import smtplib
from email.message import EmailMessage

sender_email = "guardiansurveillance.ai@gmail.com"
sender_password = "t2vqh97u"
receiver_email = "joey.yang1997@hotmail.com"
starttls_port = 587
smtp_server = "smtp.gmail.com"

msg = EmailMessage()
msg["X-Priority"] = "1"
msg["From"] = sender_email
msg["To"] = receiver_email
msg['Subject'] = "Urgent!"
msg = "test"

server = smtplib.SMTP(smtp_server, starttls_port)
server.starttls()
server.login(sender_email, sender_password)
server.sendmail(sender_email, receiver_email, msg)
server.quit()
