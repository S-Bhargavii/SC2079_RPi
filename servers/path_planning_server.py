"""
    THIS IS THE MAIN CODE FOR TASK 1.
    USE THIS.
"""
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
from utility.consts import STM32_PREFIXES_TASK1, FIN, SNAP, TASK1_IMAGE_IDS

class Task1:
    def __init__(self):
        """
            Initialises and configures logger
        """
        self.logger = logging.getLogger("Task1")
        configure_logger(self.logger)
    
    def create(self):
        """
            Create client objects
        """
        self.android = AndroidLink()
        self.stm = STMLink()
        self.logger.debug("Created client objects.")
    
    def connect(self):
        """
            Establish connection with servers in parallel.
            TO DO: handle error that arise during connection
        """
        android_p = Thread(target=self.android.connect, name="android conn.")
        stm_p = Thread(target=self.stm.connect, name="stm conn.")

        connection_threads = [stm_p, android_p]

        # start all threads
        self.logger.info("Starting connection threads ....")
        for connection_thread in connection_threads:
            connection_thread.start()
            self.logger.debug(f"Started connection thread: {connection_thread.name}")
        
        # wait for threads to finish
        for connection_thread in connection_threads:
            connection_thread.join()
            self.logger.debug(f"Connection established successfully: {connection_thread.name}")
        
        self.logger.info("Connections established successfully")
    
    def disconnect(self):
        """
            Disconnect from the servers
        """
        self.logger.info("Disconnecting...")
        self.disconnect_zmq_clients.set()
        clients = {
            "android":self.android,
            "stm":self.stm
        }
        for name, client in clients.items():
            try:
                self.logger.info(f"Disconnecting from {name}.")
                client.disconnect()
            except Exception as e:
                self.logger.error(f"Error occured when trying to disconnect from {name} : {e}")

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
            # create all shared objects
            self.create_shared_obj()

            self.proc_android_sender = Process(target=self.send_android, name="android sender") # sends messages to android
            self.proc_android_reciever = Process(target=self.recv_android, name="android reciever") # recieve messages from android
            self.proc_stm_reciever = Process(target=self.recv_stm, name="stm reciever")
            self.proc_cmd_follower = Process(target=self.cmd_follower, name="cmd follower")
            
            # start all processes
            self.proc_android_reciever.start()
            self.proc_android_sender.start()
            self.proc_cmd_follower.start()
            self.proc_stm_reciever.start()

            self.logger.info("Child processes started")
            self.android_msgs.put(AndroidMessage("info", "Robot is ready!"))

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
        self.android_msgs = self.manager.Queue()
        self.obstacles = self.manager.dict() # can change this to a normal list
        # self.commands = self.manager.Queue()
        self.stm_commands = self.manager.Queue()
        # self.android_commands = self.manager.Queue()
        self.path = self.manager.Queue()
        self.movement_lock = self.manager.Lock()
        self.shared_namespace = self.manager.Namespace()
        self.shared_namespace.stm_ack_msg = ""
        # self.shared_namespace.command_executed = ""
        # self.commands_executed = self.manager.Queue()
        self.disconnect_zmq_clients = self.manager.Event()
        self.imageRec_client_created = False # used in one process only - dont need manager
        self.pathPlanning_client_created = False

    def recv_android(self):
        """
            Process deals with recieving messages from android

            TO DO : don't think a continuous loop is required here 
            cuz the android will only send one message through out the run.
        """
        while not self.disconnect_zmq_clients.is_set():
            try:
                msg_json = self.android.recv()
                message = json.loads(msg_json)

                type = message.get("type")
                value = message.get("value") # list of dictionaries
                # for task 1 - the android sending obstacles signals start
                if type == "obstacles":
                    self.movement_lock.acquire()
                    self.logger.debug("Android sent list of obstacles")
                    android_obstacles_list = value['obstacles']
                    # skip the hard obstacle
                    # if len(android_obstacles_list) == 8:
                    #     android_obstacles_list = android_obstacles_list[:7]
                        
                    for obstacle in android_obstacles_list:
                        _obs_id = str(obstacle["id"])
                        _dir_id = 0
                        if obstacle["dir"] == "N":
                            _dir_id = 0
                        elif obstacle["dir"] == "E":
                            _dir_id = 2
                        elif obstacle["dir"] == "S":
                            _dir_id = 1
                        elif obstacle["dir"] == "W":
                            _dir_id = 3
                        self.obstacles[_obs_id] = {
                            "x": obstacle["x"],
                            "y": obstacle["y"], 
                            "id": obstacle["id"],
                            "d":_dir_id
                        }
                        # self.logger.debug(obstacle)
                        # self.logger.debug(self.obstacles[_obs_id])
                        # request path from algo

                    self.logger.debug(f"List of obstacles being sent to the path planning server : {list(self.obstacles.values())}")
                    self._request_path(list(self.obstacles.values()))
   
                elif type == "start":
                    self.movement_lock.release()
                    self.logger.info("Robot started moving.")
                    self.android_msgs.put(AndroidMessage("status", "Robot started moving"))

                else:
                    error_msg = f"An invalid message has been sent by the android : {message}"
                    self.logger.error(error_msg)
                    self.android_msgs.put(AndroidMessage("error", error_msg))
                    raise Exception(error_msg)

                # check api
                # self._check_api() ## TRUST

            except Exception as e:
                # the individual methods will add error msgs to android queue, 
                # doesnt need to be handled here :)
                self.logger.error(f"Error occured in starting the robot : {e}")
        
        self._request_path(disconnect = True)

    def send_android(self):
        """
            Send messages to android
        """
        while True:
            try:
                message = self.android_msgs.get_nowait()
                if message != None:
                    self.logger.debug(f"Sending message to android: {message}")
                    self.android.send(message)
                else:
                    pass
            except queue.Empty:
                # queue is just empty, dont raise error, continue
                continue

    def _request_path(self, data = [], robot_x = 1, robot_y = 1, robot_dir = 0, retrying= False, disconnect = False):
        """
            Requests path to the path planning algorithm
            Args:
                data - list of obstacles (dicts)
                robot_x - current x position of the robot
                robot_y - current y position of the robot
                robot_dir - current orientation of the robot
                retrying 
        """
        # create the client in the process that requires it
        if not self.pathPlanning_client_created:
            self.pathPlanner = PathPlannerZMQ()
            self.pathPlanner.connect()
            self.pathPlanning_client_created = True

        self.logger.info("Requesting path from algo")
        self.android_msgs.put(AndroidMessage("info", "Requesting path from algo"))

        if not disconnect:
            try:
                self.logger.debug("Sending message to path planning server.")
                path_planning_info = {
                    "obstacles":data,
                    "robot_x": robot_x,
                    "robot_y": robot_y, 
                    "robot_dir": robot_dir
                }

                self.logger.debug(f"Information being sent to path planning server is : {path_planning_info}")
                self.pathPlanner.send(PathPlanningMessage("path", path_planning_info))

                result = self.pathPlanner.recv()
                self.logger.info("Recieved path from algo.")
            except Exception as e:
                # raised when something went wrong or timeout when requesting images
                self.logger.error(
                    f"Something went wrong when requesting path from Algo API: {e}")
                self.android_msgs.put(AndroidMessage(
                    "error", "Something went wrong when requesting path from Algo API."))
                # early return when not able to get response from the path planning server
                raise e 

            stm_cmds = result["stm_commands"]
            android_cmds = result["android_commands"]
            path = result["path"]

            self.logger.debug(f"STM Commands recieved from API: {stm_cmds}")
            self.logger.debug(f"ANDROID commands recieved from API: {android_cmds}")
            self.logger.debug(f"Path recieved from API: {path}")

            self._clear_queues()
            for c in stm_cmds:
                self.stm_commands.put(c)
            # for c in android_cmds:
            #     self.android_commands.put(c)
            # for p in path:
            #     self.path.put(p)
            
            self.logger.info(
                "Commands and path received Algo API. Robot is ready to move.")
            self.android_msgs.put(AndroidMessage(
                "info", "Commands and path received Algo API. Robot is ready to move."))
        else:
            self.pathPlanner.disconnect()
        
    def _clear_queues(self):
        """
            Clear both path and command queues
        """
        # while not self.commands.empty():
        #     self.commands.get()
        # while not self.path.empty():
        #     self.path.get()
        while not self.stm_commands.empty():
            self.stm_commands.get()
        # while not self.android_commands.empty():
        #     self.android_commands.get()

    def recv_stm(self):
        """
            Deals with release and aquiring the robot lock
        """
        while True:
            message:str = self.stm.recv()
            try:
                if message.startswith(STM32_PREFIXES_TASK1):
                    self.movement_lock.release()
                    self.logger.debug("ACK has been received, robot lock has been released. Can move now.")
                    # command_executed = self.commands_executed.get_nowait()
                    # cur_command_executed = self.android_commands.get_nowait()
                    # self.android_msgs.put(
                    #     AndroidMessage('location',{
                    #         "command": cur_command_executed
                    #     })
                    # )
                else:
                    self.logger.warning(f"Unknown message recieved from STM: {message}")
            except Exception as e:
                self.logger.error(f"Failed to recieve lock: {e}")
                raise e

    def _pad_zeros(self, command: str):
        """Pad with leading zeros to ensure a 3-character argument."""
        return f"{'0' * (3 - len(command))}{command}"

    def _pad_sign(self, command: str):
        """Pad with '+' or '0' to make the argument 3 characters long."""
        if len(command) == 3:
            return command  # Already correct length
        
        if command == '0':
            return "000"

        if command[0] == '-':
            return f"-0{command[1:]}"  # Ensure negative values are 3 characters
        else:
            return f"+{'0' * (2 - len(command))}{command}"  # Pad with '+' for positive

    def process_command(self, command: str):
        """Processes the command string by padding only the 2nd and 3rd arguments."""
        result = command[:2]  # Extract the command type (e.g., "T")
        arguments = command[2:].split('|')  # Split arguments by '|'

        processed_args = [arguments[0]]  # Keep the first argument unchanged

        if len(arguments) > 1:
            processed_args.append(self._pad_sign(arguments[1]))  # Apply padding to 2nd arg
        if len(arguments) > 2:
            processed_args.append(self._pad_zeros(arguments[2]))  # Apply padding to 3rd arg

        return result + '|'.join(processed_args)
    

    def cmd_follower(self):
        command: str = ""
        self.image_client(connect=True)
        while not self.disconnect_zmq_clients.is_set():

            try:
                command = self.stm_commands.get_nowait()
                # if command != None:
                #     # self.logger.debug(f"following command: {command}")
                # else:
                #     continue
            except queue.Empty:
                # queue is just empty, dont raise error, continue
                continue

            try:
                self.logger.debug("Trying to acquire robot lock...")
                self.movement_lock.acquire()
                self.logger.debug("Acquired robot lock.")

                # stm32 commands - send to stm
                if command.startswith(STM32_PREFIXES_TASK1):
                    command = self.process_command(command)
                    self.stm.send(command)
                    # self.commands_executed.put(command)
                    self.logger.debug(f"Sent command to STM: [{command}]")
                # snap command
                elif command.startswith(SNAP):
                    self.logger.debug("Command to be executed is a SNAP command.")
                    image_id = self.image_client(True)
                    image_char = "-1"
                    try:
                        image_char = TASK1_IMAGE_IDS[image_id]
                    except:
                        image_char = "-1"
                    obstacle_id = command[4]
                    obstacle_id_str = str(obstacle_id)
                    direction = self._get_direction_string(self.obstacles[obstacle_id_str]["d"])
                    # msg_to_android = f"TARGET,B{obstacle_id},{image_char},{direction}"
                    msg_to_android = f"TARGET,B{obstacle_id},{image_id},{direction}"
                    self.android_msgs.put(AndroidMessage("image-rec", msg_to_android))
                    self.movement_lock.release()
                # finish run
                elif command == FIN:
                    self.logger.debug("FIN command.")
                    self.logger.info("Finished executing commands.")
                    time.sleep(2)
                    self.android_msgs.put(AndroidMessage("status", "Finished"))
                    self.movement_lock.release()
                    self.image_client(False)
                else:
                    raise Exception(f"Message of unknown format recieved: {command}")

            except Exception as e:
                self.logger.error("Error in executing commands: {e}")
                self.android_msgs.put(AndroidMessage("error", f"Error occured when executing command :{command}"))
                raise e
        
        self.image_client(disconnect=True)
        
    #  use if-else instead of match because the python version on RPi doesnt support it
    def _get_direction_string(self,dir_int:int)->str:
        if dir_int == 0:
            dir_str = "N"
        elif dir_int == 2:
            dir_str = "E"
        elif dir_int == 1:
            dir_str = "S"
        elif dir_int == 3:
            dir_str = "W"    
        return dir_str
    
    def _get_direction_int(self, dir_str:str)->int:
        if dir_str == "N":
            dir_int = 0 
        elif dir_str == "E":
            dir_int = 2
        elif dir_str == "S":
            dir_int = 1
        elif dir_str == "W":
            dir_int = 3
        return dir_int

    def image_client(self, snap_and_rec = True, disconnect = False, connect=False):
        if connect and not self.imageRec_client_created:
            self.imageRec = ImageRecLink()
            self.imageRec.connect()
            self.imageRec_client_created = True
            return 

        if disconnect:
            self.logger.debug("Disconnecting from imagezmq server.")
            self.imageRec.disconnect()
            return
        
        if snap_and_rec:
            self.logger.debug("Snap and Rec requested to the image zmq server.")
            image_id = self.imageRec.imageRec("task_1")
            if image_id != 0:
                return image_id
            else:
                self.logger.error("Somthing wrong happened when requested for snap and rec.")
                pass
            return
        
        if not snap_and_rec:
            self.logger.debug("Stitch requested to the image zmq server.")
            ret_code = self.imageRec.imageRec("stitch")
            if ret_code == 3:
                self.logger.info("Images stitched!")
                return
            else:
                self.logger.error("Something wrong happened when requested for image stitch.")
                pass
            return 
    
    def stop(self):
        """
            Stop the run
        """
        # disconnect from all the servers
        self.disconnect()

        running_procs = [
            self.proc_android_reciever, 
            self.proc_android_sender, 
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
    task1_executor = Task1()
    task1_executor.start()