from utility.logger import configure_logger
import logging
from abc import ABC, abstractmethod
from typing import Optional

class Link(ABC):
    """
        Abstract class to handle communication on links 
        This class is required for - 
        1. RPi and Android communication 
        2. RPi and STM communication
    """
    def __init__(self, name):
        """
            Initialises and configures the logger for the link class
        """
        self.logger = logging.getLogger(name)
        configure_logger(self.logger)
    
    @abstractmethod
    def send(self, message:str) -> None:
        """
            Method for sending messages from RPi
        """
        pass

    @abstractmethod
    def recv(self) -> Optional[str]:
        """
            Method for recieving messages to RPi
        """
        pass
