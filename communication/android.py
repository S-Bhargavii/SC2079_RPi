import bluetooth
import json 
import os 
import socket
from typing import Optional 
from communication.link import Link

class AndroidMessage:
    """
        Class for construction message to communicate with Android tablet
    """
    def __init__(self, category, value):
        self._category = category
        self._value = value
    
    @property
    def category(self):
        return self._category
    
    @property
    def value(self):
        return self._value

    @property
    def jsonify(self) -> str:
        return json.dumps({"cat":self._category, "value":self._value}) # JSON strings are universally understood
    
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
            os.system("sudo hciconfig hci0 piscan")

            #Initialise the server socket - this is the listening socket and it will listen for connections 
            self.server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM) # creating an object to use its methods
            self.server_socket.bind(("", bluetooth.PORT_ANY)) # (ip address, port) for binding the server socket to - it will listen on this port - "" refers to all available networks
            self.server_socket.listen(1) # the number of elements in the waiting queue - anything that comes after the waiting queue is full is rejected

            port = self.server_socket.getsockname()[1] # returns the exact port that was assigned 
            uuid = "a7hb421j-0g47-jk12-i98g-ac64li8fqb7m"

            bluetooth.advertise_service(self.server_socket, "MDPGrp3-bluetooth", service_id=uuid, service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS], profiles=bluetooth.SERIAL_PORT_PROFILE)
            self.logger.info(f"Waiting for bluetooth connection request on port : {port}")
            self.client_socket, client_info = self.server_socket.accept()
            self.logger.info(f"Initiated communication with the client : {client_info}")

        except Exception as e:
            self.logger.error(f"Error in bluetooth connection :{e}")
            if self.server_socket:
                self.server_socket.close() # close the server socket if it has been initialised
            if self.client_socket:
                self.client_socket.close()  # close the server socket if it has been initialised

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

    def recv(self, message:AndroidMessage):
        """
            Recieve a message from the android
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
