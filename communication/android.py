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
    
class AndroidLink(Link):
    def __init__(self):
        super().__init__("android")
        self.server_socket = None # this is just a listening socket - it just listens to the incoming connection requests
        self.client_socket = None # this is the socket meant for communication with the client

    def connect(self):
        """
            Method to connect to android by bluetooth
        """
        self.logger.info("Bluetooth connection started")
        try:
            os.system("sudo hciconfig hci0 piscan") # allows the rpi bluetooth to be discoverable

            #Initialise the server socket - this is the listening socket and it will listen for connections 
            self.server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM) # creating an object to use its methods
            self.server_socket.bind(("", bluetooth.PORT_ANY)) # (ip address, port) for binding the server socket to - it will listen on this port - "" refers to all available networks
            port = self.server_socket.getsockname()[1] # returns the exact port that was assigned 
            # use os command
            os.system(f"sudo sdptool add --channel={port} SP")
            self.server_socket.listen(1) # the number of elements in the waiting queue - anything that comes after the waiting queue is full is rejected

            self.logger.info(f"Waiting for bluetooth connection request on port : {port}")
            self.client_socket, client_info = self.server_socket.accept()
            self.logger.info(f"Initiated communication with the client : {client_info}")

        except Exception as e:
            self.logger.error(f"Error in bluetooth connection :{e}")
            if self.server_socket:
                self.server_socket.close() # close the server socket if it has been initialised
            if self.client_socket:
                self.client_socket.close()  # close the server socket if it has been initialised
            raise e  # raise error 

    def disconnect(self):
        """
            Disconnect from Android Bluetooth connection
        """
        try:
            self.logger.debug("Disconnecting Bluetooth link")
            self.server_socket.shutdown(socket.SHUT_RDWR)
            self.server_socket.close()
            self.server_socket = None
            self.client_socket.shutdown(socket.SHUT_RDWR)
            self.client_socket.close()
            self.client_socket = None
            self.logger.info("Disconnected Bluetooth link")
        except Exception as e:
            self.logger.error(f"Failed to disconnect Bluetooth link: {e}")
            raise e # raise error 

    def send(self, message: AndroidMessage):
        """
            Send a message to the Android
        """
        try:
            self.client_socket.send(f"{message.jsonify}\n".encode("utf-8")) # must be in bytes to be transfered over
            self.logger.debug(f"Send to Android : {message}")
            return
        except Exception as e:
            self.logger.error(f"Failed to send message : {e}")
            raise e 

    def recv(self):
        """
            Recieve a json object from the android in JSON string format
        """
        try:
            tmp = self.client_socket.recv(1024) # 1024 is the buffer size that can be received at once
            # self.logger.debug(tmp)
            message = tmp.strip().decode("utf-8") # decode from bytes to understandable characters
            self.logger.debug(f"Recieved message from Android: {message}")
            return message
        except Exception as e:
            self.logger.error(f"Failed to recieve message : {e}")
            raise e
