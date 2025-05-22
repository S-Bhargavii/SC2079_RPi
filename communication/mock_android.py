import bluetooth
import json 
import os 
import socket
from typing import Optional 
from communication.link import Link
from utility.settings import UUID

class AndroidMessage:
    """
        Class for construction message to communicate with Android tablet
    """
    def __init__(self, type:str, value):
        self._type = type
        self._value = value
    
    @property
    def type(self):
        return self._type
    
    @property
    def value(self):
        return self._value

    @property
    def jsonify(self) -> str:
        return json.dumps({"type":self._type, "value":self._value}) # JSON strings are universally understood
    
    def __str__(self):
        return f"AndroidMessage(type={self._type}, value={self._value})"
    
    def __repr__(self):
        return f"AndroidMessage(type={self._type}, value={self._value})"
    
class MockAndroidLink(Link):
    def __init__(self):
        super().__init__("android")
        self.server_socket = None # this is just a listening socket - it just listens to the incoming connection requests
        self.client_socket = None # this is the socket meant for communication with the client

    def connect(self):
        """
            Method to connect to android by bluetooth
        """
        self.logger.info("Connected to bluetooth")
        return True

    def disconnect(self):
        """
            Disconnect from Android Bluetooth connection
        """
        self.logger.info("Disconnected to bluetooth")
        return True

    def send(self, message: AndroidMessage):
        """
            Send a message to the Android
        """
        self.logger.info(f"Sent message to android : {message}")
        return True

    def recv(self):
        """
            Recieve a json object from the android in JSON string format
        """
        return {"message":"hello"}
