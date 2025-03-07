import socket
from communication.link import Link
from utility.settings import PATH_API_IP, PATH_API_PORT, PATH_API_RESPONSE_TIMEOUT, PATH_API_CHECK_CONNECTION_TIMEOUT
import zmq 

class PathPlanningMessage:
    """
        Class for construction message to communicate with Path Planning api
        category - pass the category of the msg you are sending - this is required to know how to handle requests on the server side.
        value - the message you want to send in dictionary format. 
    """
    def __init__(self, category, value:object={}):
        self._category = category # this is required to know how to handle requests on the server side
        self._value = value
        self.msg = {"cat":self._category}
        self.msg.update(self._value) 
    
    @property
    def category(self):
        return self._category
    
    @property
    def value(self):
        return self._value
    
    def __str__(self):
        return f"AndroidMessage(type={self._type}, value={self._value})"
    
    def __repr__(self):
        return f"AndroidMessage(type={self._type}, value={self._value})"
    
class PathPlannerZMQ(Link):
    """
        Class to handle communication with the path planning algorithm
    """
    def __init__(self):
        super().__init__("PathPlanningZMQ")
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)

    def connect(self):
        """
            Establishes connection to the path planning server
        """
        try:
            address = f"tcp://{PATH_API_IP}:{PATH_API_PORT}"
            self.socket.connect(address)
            self.logger.info("Established connection with the path planning server.")
        except Exception as e:
            self.logger.error("Error in connecting to the Path Planning Server: {e}")
            raise e # raise error to let main thread know
    
    def check_connection(self):
        """
            Checks if the server is running and if its able to connect
            This is required before starting the race.
            Returns:
                bool: True if the server is running
        """
        try:
            self.socket.setsockopt(zmq.RCVTIMEO, PATH_API_CHECK_CONNECTION_TIMEOUT) # wait 0.5 seconds for the test response
            self.send(PathPlanningMessage("test"))
            self.recv()
            #  an error is raised in both these methods if something goes wrong,
            # so handling in try catch is fine
            self.logger.info("Path planning connection server is active.")
            return True
        except:
            return False
    
    def set_timeout(self):
        """
            Set socket timeout
            TO DO : i think its a bit wasteful to have a new method 
            for this -- will see where this needs to be placed
        """
        self.socket.setsockopt(zmq.RCVTIMEO, PATH_API_RESPONSE_TIMEOUT) # change settings to wait for longer

    def disconnect(self):
        """
            Disconnect from the path planning server.
        """
        try:
            disconnect_msg = PathPlanningMessage("disconnect")
            self.send(disconnect_msg)
            self.socket.close()
            self.context.term()
            self.logger.info("Disconnected from the path planning server.")
        except Exception as e:
            self.logger.error("Error occured when trying to disconnect from server {e}")
            raise e
    
    def send(self, message: PathPlanningMessage):
        """
            This message sends messages over to the path planning server.
            Args:
                message: PathPlanningMessage. 
        """
        try:
            self.socket.send_json(message.msg)
            return
        except Exception as e:
            self.logger.error("Error occured while trying to send message: {e}")
            raise e 

    def recv(self):
        """
            Recieves message from the path planning server
            Returns:
                PathPlanningMessage: It will be in the form of a dictionary.
                {"cat":"smthg", key value pairs follow}
                TO CHECK - idk y i wrote this, it is returning a dictionary, check later
        """
        try:
            message = self.socket.recv_json()
            self.logger.debug(f"Recieved from Path planning server: {message}")
            return message
        except Exception as e:
            # intercept to just print the error message and then continue to raise error
            self.logger.error(f"Error occured while recieving message from Path Planning server: {e}")
            raise e