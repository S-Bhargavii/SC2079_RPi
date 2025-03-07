# this is the image recognition task - WEEK 9
import logging
import queue
import json
from utility.logger import configure_logger
from communication.android import AndroidLink, AndroidMessage
from communication.imageRec import ImageRecLink
from communication.stm32 import STMLink
from multiprocessing import Manager, Process

class Task2:
    def __init__(self):
        # preapre and configure logger
        self.logger = logging.getLogger("Task1")
        configure_logger(self.logger)

    def _create_and_connect(self):
        """
            Create clients and connect to all the servers
        """
        # create objects for communication
        self.android = AndroidLink()
        self.imageRec = ImageRecLink()
        # self.pathPlannerZMQ = PathPlannerZMQ()
        self.stm = STMLink()

        # start all the connections 
        self.android.connect()
        self.imageRec.connect()
        # self.pathPlannerZMQ.connect()
        self.stm.connect()

        self.first_direction = None # either left or right 
        self.second_direction = None # either left or right

    def _disconnect(self):
        """
            Disconnect from all the servers
        """
        self.android.disconnect()
        self.imageRec.disconnect()
        self.stm.disconnect()
        # self.pathPlannerZMQ.disconnect()

    def _create_manager(self):
        """ 
            Creates a manager and objects to be shared among the processes
        """
        # Manager to manage all the python objects 
        self.manager = Manager()

        # message queues - these objects are shared between all the processes
        self.android_queue = self.manager.Queue()
        self.rpi_action_queue = self.manager.Queue()
        self.command_queue = self.manager.Queue()
        # self.path_queue = self.manager.Queue()
        
    def _start_proc(self):
        """
            Start processes to recieve and send messages 
        """
        pass

    # ANDROID
    def _recv_android(self):
        """
            Recieves messages from android - andorid is only used for starting the robot
        """
        while True:
            #  note - the program is paused here until it recieves some message
            msg_str = self.android.recv() # json object 
            message = json.loads(msg_str) # convert to python object
            
            if message["cat"] == "control":
                self.logger.debug("Android has sent a control command")
                # to start the robot, message must be like 
                # { 
                #       "cat":"control", "value":"start"
                # }
                if message["value"] == "start":
                    if not self._check_api():
                        self.logger.error("ZMQ servers are down")
                        self.android_queue.put(AndroidMessage("error","API is down, abort start command"))
                    else:
                        self.stm_link.send("start")
                        self.logger.info("Robot started moving")
                        self.first_direction = self._snap_and_rec()
                        self.logger.info(f"Picture with id {self.first_direction} has been taken")
                        self.android_queue.put(AndroidMessage("status", "Robot started moving"))
            else:
                self.logger.error("A valid message has not been sent by the ANDROID")
                self.android_queue.put(AndroidMessage("error", "Message of wrong format has been sent"))

    def _send_android(self):
        """ 
            Send messages to android
            NOTE - this is not really required in this task
        """
        while True:
            try:
                # keep checking what if there is any messages that 
                # are required to be sent to Android 
               message = self.android_queue.get_nowait()
            except queue.Empty:
                continue
            
            self.android_link.send(message) # to do : error handling when this gives an error

    # STM
    def _recv_stm(self):
        """
            Recieves first acknowledgement message from the STM and then controls the stm thereafter
        """
        pass
    
    def _send_stm(self):
        """
            Deals with sending the stm the commands that are recieved 
            from the path planning algorithm
        """
        pass

    # IMAGE RECOGNITION
    def _snap_and_rec(self):
        """ 
            Snaps a picture and sends it over to the image recognition server 
            to identify image. Returns the image id. 
        """
        image_id = self.imageRec.imageRec("task_2")
        return image_id

    def _check_api(self):
        return self.pathPlannerZMQ.check_connection() and self.imageRec.check_connection()
