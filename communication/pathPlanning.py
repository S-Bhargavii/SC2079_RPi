from typing import Optional
import socket
from communication.link import Link
from utility.settings import PATH_API_IP, PATH_API_PORT

class PathPlanner(Link):
    def __init__(self):
        super.__init__("Path Planning Algorithm")
        self.host = PATH_API_IP
        self.port = PATH_API_PORT
        self.connected = False
        self.server_socket = None
        self.client_socket = None
    
    def connect(self):
        """
            Connect using TCP socket to the Path Planning Algorithm
        """
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.logger.debug(f"Socket established successfully")

        try:
            self.server_socket.bind((self.host, self.port))
            self.logger.debug(f"Socket binded to {self.host}:{self.port}")
        except Exception as e:
            self.logger.error("Error occured when trying to connect to Path Planning Algorithm: {e}")
            self.server_socket.close()
            return

        # Establish connection to PC
        try:
            self.server_socket.listen(1) #128 in repository
            self.client_socket, client_address = self.server_socket.accept()
            self.logger.info(f"Connected to Path Planning Algorithm at {client_address}")
        except Exception as e:
            self.logger.error("Error occured when trying to connect to Path Planning Algorithm: {e}")
            self.server_socket.close()
            return
        
        self.connected = True

    def disconnect(self):
        """
            Close the tcp port and disconnect from the Path Planning Algorithm
        """
        try:
            self.server_socket.shutdown(socket.SHUT_RDWR)
            self.client_socket.shutdown(socket.SHUT_RDWR)
            self.server_socket.close()
            self.client_socket.close()
            self.server_socket = None
            self.client_socket = None   
            self.connected = False
            self.logger.info("Disconnected from Path Planning Algorithm")
        except Exception as e:
            self.logger.error("Error occured when trying to close connection: {e}")
            return
    
    def send(self, message:str) -> None:
        try:
            message_bytes = f"{message}".encode("utf-8")
            self.client_socket.sendall(message_bytes)
            self.logger.info(f"Sent to Path Planning Algorithm: {message}")
            return
        except Exception as e:
            self.logger.error("Error occured while trying to send message: {e}")
            raise e  
    
    def recv(self):
        try:
            message = self.client_socket.recv(1024).decode("utf-8")
            self.logger.info(f"Recieved from Path Planning Algorithm: {message}")
            return message
        except Exception as e:
            self.logger.error(f"Error occured while recieving message from Path Planning algorithm: {e}")
            raise e