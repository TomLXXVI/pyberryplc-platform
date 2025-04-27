import smtplib
import ssl
from email.message import EmailMessage
import threading


class EmailNotification:

    def __init__(
        self,
        smtp_server: str,
        port: int,
        sending_address: str,
        password: str,
        receiving_address: str,
        subject: str,
        max_retries: int = 5
    ):
        self.smtp_server = smtp_server
        self.port = port
        self.sending_address = sending_address
        self.password = password
        self.receiving_address = receiving_address
        self.subject = subject
        self.max_retries = max_retries
        self._ssl_context = ssl.create_default_context()

    def _send(self, content: str):
        msg = EmailMessage()
        msg['Subject'] = self.subject
        msg['From'] = self.sending_address
        msg['To'] = self.receiving_address
        msg.set_content(content)
        server = None
        for i in range(self.max_retries):
            try:
                server = smtplib.SMTP(host=self.smtp_server, port=self.port, timeout=5)
                server.starttls(context=self._ssl_context)
                server.login(self.sending_address, self.password)
                server.send_message(msg)
            except Exception as error:
                if i < self.max_retries - 1:
                    continue
                else:
                    raise error
            else:
                break
            finally:
                if server: server.quit()

    def send(self, content: str):
        thread = threading.Thread(target=self._send, args=(content,))
        thread.start()


if __name__ == '__main__':

    from decouple import config

    eml_notification = EmailNotification(
        smtp_server='smtp.gmail.com',
        port=587,
        sending_address='unipi.plc@gmail.com',
        password=config('PASSWORD', default=''),
        receiving_address='tom.chr@proximus.be',
        subject='test'
    )

    eml_notification.send(content='test')
