import logging 
import queue
import json 
import sys
import time
sys.path.append("/home/pi/Desktop/RPi")
from utility.logger import configure_logger
from communication.android import AndroidLink, AndroidMessage
from communication.imageRec import ImageRecLink
from communication.pathPlanning import PathPlannerZMQ, PathPlanningMessage
from communication.stm32 import STMLink
from multiprocessing import Manager, Process
from threading import Thread
from utility.consts import FIN, SNAP, TASK1_IMAGE_IDS
BULLSEYE_ID = -1
RIGHT_TURN_1_COMMANDS = ["FL90", "FR90", "FW27", "BL90", "FW23", "SNAP"]
RIGHT_TURN_2_COMMANDS = ["FL90", "FR90", "FW30", "BL90", "FW30", "SNAP"]
RIGHT_TURN_3_COMMANDS = ["FL90", "FR90", "FW30", "BL90", "FW30", "SNAP"]

IMAGE_REC_CODE = "task_1"
ACK_MSG = "ACK"

class CheckList:
    def __init__(self):
        """
            Initialises and configures logger
        """
        self.logger = logging.getLogger("Checklist")
        configure_logger(self.logger)
    
    def create(self):
        """
            Create client objects
        """
        self.stm = STMLink()
        self.logger.debug("Created client objects.")
    
    def connect(self):
        """
            Establish connection with STM.
        """
        try:
            self.stm.connect()
            self.logger.info("Connections establish successfully.")
        except Exception as e:
            self.logger.error(f"Failed to establish connection with STM : {e}")
    
    def disconnect(self):
        """
            Disconnect from STM
        """
        try:
            self.stm.disconnect()
            self.logger.info("Disconnected from STM.")
        except Exception as e:
            self.logger.error(f"Error occured when trying to disconnect from STM : {e}")

    def start(self):
        """
            Start the child processes.

            Purpose - when you create a child process, add it here
        """      
        try:
            # create all clients 
            self.create()
            # connect to all clients
            self.connect()

            # self.stm.recv()
            self.logger.debug("Finished starter commands.")
            # create all shared objects
            self.create_shared_obj()

            self.commands.put("SNAP")

            self.proc_stm_reciever = Process(target=self.recv_stm, name="stm reciever")
            self.proc_cmd_follower = Process(target=self.cmd_follower, name="cmd follower")
            
            # start all processes
            self.proc_cmd_follower.start()
            self.proc_stm_reciever.start()

            self.logger.info("Child processes started")

            while True:
                time.sleep(0.5) # to keep alive

        except KeyboardInterrupt:
            self.logger.info("Ctrl C pressed....stopping")
            self.stop()
            
    def create_shared_obj(self):
        """
            This deals with creating shared objects

            Purpose - when you create objects that are to be shared across multiple processes, add it here
        """
        self.manager = Manager()
        self.movement_lock = self.manager.Lock()

        self.shared_namespace = self.manager.Namespace()
        self.shared_namespace.bullseye = 0
        self.shared_namespace.command_executed = ""
        self.commands = self.manager.Queue()

        self.disconnect_zmq_clients = self.manager.Event()
        self.imageRec_client_created = False # used in one process only - dont need manager
    
    def recv_stm(self):
        """
            Deals with release and aquiring the robot lock
        """
        while True:
            message:str = self.stm.recv()
            try:
                if message.startswith(ACK_MSG):
                    time.sleep(2)
                    self.movement_lock.release()
                    self.logger.debug("ACK has been received, robot lock has been released. Can move now.")
                else:
                    self.logger.warning(f"Unknown message recieved from STM: {message}")
            except Exception as e:
                self.logger.error(f"Failed to recieve lock: {e}")
                raise e

    def cmd_follower(self):
        command: str = ""
        while not self.disconnect_zmq_clients.is_set():

            try:
                command = self.commands.get_nowait()
                if command != None:
                    self.logger.debug(f"following command: {command}")
                else:
                    continue
            except queue.Empty:
                # queue is just empty, dont raise error, continue
                continue

            try:
                self.logger.debug("Trying to acquire robot lock...")
                self.movement_lock.acquire()
                self.logger.debug("Acquired robot lock.")

                # stm32 commands - send to stm
                if not command.startswith(SNAP):
                    self.stm.send(command)
                    self.shared_namespace.command_executed = command
                    self.logger.debug(f"Sent command to STM: {command}")
                # snap command
                elif command.startswith(SNAP):
                    self.logger.debug("Command to be executed is a SNAP command.")
                    image_id = self.image_client(snap_and_rec=True)
                    if image_id == BULLSEYE_ID:
                        self.shared_namespace.bullseye += 1
                        self.right_turn()
                    else:
                        self.image_found(image_id=image_id)
                    self.logger.debug("Recieve lock from image rec server")
                    self.movement_lock.release()
                else:
                    raise Exception(f"Message of unknown format recieved: {command}")

            except Exception as e:
                self.logger.error("Error in executing commands: {e}")
                # self.android_msgs.put(AndroidMessage("error", f"Error occured when executing command :{command}"))
                raise e
        
        self.image_client(disconnect=True)

    def right_turn(self):
        """
            Adds list of commands to the command queue, that are required by the STM to take a right turn.
            This must be followed by SNAP to take picture
        """
        self.logger.info(f"Image not found !!!")
        if self.shared_namespace.bullseye == 1:
            for command in RIGHT_TURN_1_COMMANDS:
                self.commands.put(command)
        elif self.shared_namespace.bullseye == 2:
            for command in RIGHT_TURN_2_COMMANDS:
                self.commands.put(command)
        else:
            for command in RIGHT_TURN_3_COMMANDS:
                self.commands.put(command)
        # self.movement_lock.release()
        return
    
    def image_found(self, image_id):
        """
            What to do when the image is found.
        """
        self.logger.info(f"IMAGE found with id : {image_id} charachter: {TASK1_IMAGE_IDS[image_id]}")
        # self.commands.put("K")
        return

    def image_client(self, snap_and_rec = True, disconnect = False):
        if not self.imageRec_client_created:
            self.imageRec = ImageRecLink()
            self.imageRec.connect()
            self.imageRec_client_created = True

        if disconnect:
            self.logger.debug("Disconnecting from imagezmq server.")
            self.imageRec.disconnect()
            return
        
        if snap_and_rec:
            self.logger.debug("Snap and Rec requested to the image zmq server.")
            image_id = self.imageRec.imageRec(IMAGE_REC_CODE)
            # what is passed when a bullseye is given?
            if image_id != 0:
                return image_id
            else:
                self.logger.error("Somthing wrong happened when requested for snap and rec.")
                pass
            return
    
    def stop(self):
        """
            Stop the run
        """
        # disconnect from all the servers
        self.disconnect()

        running_procs = [
            self.proc_cmd_follower, 
            self.proc_stm_reciever
        ]

        for proc in running_procs:
            if proc and proc.is_alive():
                self.logger.info(f"Terminating {proc.name}")
                proc.terminate()
        
        for proc in running_procs:
            if proc:
                proc.join()
                self.logger.info(f"{proc.name} has been stopped.")

        self.logger.info("All processes terminated successfully.")
    
if __name__ == "__main__":
    task1_executor = CheckList()
    task1_executor.start()