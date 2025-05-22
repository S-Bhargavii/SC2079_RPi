from typing import Optional
import serial
from communication.link import Link
from utility.settings import SERIAL_PORT, BAUD_RATE

class STMLink(Link):
    def __init__(self):
        super().__init__("STM")
        self.serial_link = None
    
    def connect(self):
        """
            Connect using UART connections
        """
        try:
            self.serial_link = serial.Serial(SERIAL_PORT, BAUD_RATE)
            self.logger.info("Connected to STM32")
        except Exception as e:
            self.logger.error("Error occured when trying to connect to STM: {e}")
            raise e # raise error to let main thread know

    def disconnect(self):
        """
            Close the serial link
        """
        try:
            self.serial_link.close()
            self.serial_link = None
            self.logger.info("Disconnected from STM32")
        except Exception as e:
            self.logger.error("Error occured when trying to close connection: {e}")
            raise e
    
    def send(self, message:str) -> None:
        """
            Send messages to STM32.
            Args:
                msg: The message that you want to send.
        """
        try:
            message_bytes = f"{message}".encode("utf-8")
            self.serial_link.write(message_bytes)
            return
        except Exception as e:
            self.logger.error("Error occured while trying to send message: {e}")
            raise e  
    
    def recv(self):
        """
            Recieve messages from STM32.
            Returns:
                message: The message sent by STM32.
        """
        try:
            message = self.serial_link.readline().strip().decode("utf-8")
            self.logger.debug(f"Recieved from STM32: {message}")
            return message
        except Exception as e:
            self.logger.error(f"Error occured while recieving message from STM32: {e}")
            raise e