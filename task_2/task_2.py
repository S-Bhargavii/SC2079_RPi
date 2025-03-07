import logging 
import queue
import json 
from utility.logger import configure_logger
from communication.android import AndroidLink, AndroidMessage
from communication.imageRec import ImageRecLink
from communication.pathPlanning import PathPlannerZMQ, PathPlanningMessage
from communication.stm32 import STMLink
from multiprocessing import Manager, Process
from threading import Thread
from utility.consts import LEFT_ARROW, RIGHT_ARROW, FIN, STM32_PREFIXES_TASK2

class Task2:
    def __init__(self):
        """
            Initialises and configures logger
        """
        self.logger = logging.getLogger("Task2")
        configure_logger(self.logger)
    
    def create(self):
        """
            Create client objects
        """
        self.android = AndroidLink()
        self.imageRec = ImageRecLink()
        self.pathPlanner = PathPlannerZMQ()
        self.stm = STMLink()
        self.logger.debug("Created client objects.")
    
    def connect(self, parallel:bool = True):
        """
            Establish connection with the servers.
            Args:
                parallel - if true multiple threads are started for connection
        """
        try:
            if parallel:
                self.connect_p()
            else:
                self.connect_s()
        except Exception as e:
            self.logger.error("Error occured while trying to connect with: {e}")
            raise e
    
    def connect_p(self):
        """
            Establish connection with servers in parallel.
            TO DO: handle error that arise during connection
        """
        android_p = Thread(target=self.android.connect, name="android conn.")
        imageRec_p = Thread(target=self.imageRec.connect, name="image conn.")
        pathPlanner_p = Thread(target=self.pathPlanner.connect, name="path conn.")
        stm_p = Thread(target=self.stm.connect, name="stm conn.")

        connection_threads = [pathPlanner_p, stm_p, android_p, imageRec_p]

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

    def connect_s(self):
        """
            Establish connection in series
        """
        clients = {
            "stm":self.stm,
            "android":self.android,
            "pathPlanner":self.pathPlanner,
            "imageRec":self.imageRec
        }
        for name, client in clients.items():
            try:
                client.connect()
                self.logger.debug(f"Established connection with {name} server")
            except Exception as e:
                self.logger.error(f"Error occured when trying to establish connection with {name}")
                raise e # program stops here
        self.logger.info("Established connection with all servers.")
    
    def disconnect(self):
        """
            Disconnect from the servers
        """
        clients = {
            "stm":self.stm,
            "android":self.android,
            "pathPlanner":self.pathPlanner,
            "imageRec":self.imageRec
        }
        for name, client in clients.items():
            try:
                client.disconnect()
            except Exception as e:
                self.logger.error(f"Error occured when trying to establish connection with {name}")
                raise e # program stops here
        self.logger.info("Disconnected from all servers.")

    def start(self):
        """
            Start the child processes.

            Purpose - when you create a child process, add it here
        """      
        try:
            self.proc_android_sender = Process(target=self.send_android, name="android sender") # sends messages to android
            self.proc_android_reciever = Process(target=self.recv_android, name="android reciever") # recieve messages from android

            self.proc_stm_reciever = Process(target=self.recv_stm, name="stm reciever")
            self.proc_cmd_follower = Process(target=self.cmd_follower, name="cmd follower")
            
            # create all clients 
            self.create()
            # connect to all clients
            self.connect()
            # create all shared objects
            self.create_shared_obj()
            # start all processes
            self.proc_android_reciever.start()
            self.proc_android_sender.start()
            self.proc_cmd_follower.start()
            self.proc_stm_reciever.start()

            self.logger.info("Child processes started")
            self.android_msgs.put(AndroidMessage("info", "Robot is ready!"))

        except KeyboardInterrupt:
            self.stop()

    def create_shared_obj(self):
        """
            This deals with creating shared objects

            Purpose - when you create objects that are to be shared across multiple processes, add it here
        """
        self.manager = Manager()
        self.android_msgs = self.manager.Queue()
        self.commands = self.manager.Queue()
        self.movement_lock = self.manager.Lock()

    def recv_android(self):
        """
            Process deals with recieving messages from android

            TO DO : don't think a continuous loop is required here 
            cuz the android will only send one message through out the run.
        """
        while True:
            try:
                msg_str = self.android.recv()
                message = json.loads(msg_str)
                cat = message["cat"]
                value = message["value"]
                if cat == "control":
                    if value == "start":
                        self._check_api()
                        self._clear_queues()
                        self.commands.put("RS00")

                        # begin to recognise the image at the start itself and then move accordingly
                        self.first_direction = self._snap_and_rec()
                        if self.first_direction == LEFT_ARROW:
                            self.commands.put("OB01")
                            self.commands.put("UL00")
                        elif self.first_direction == RIGHT_ARROW:
                            self.commands.put("OB01")
                            self.commands.put("UR00")
                        # TO DO : handle none case

                        self.logger.info("Starting robot")
                        self.commands.put(AndroidMessage("status", "Robot started running.")) #🎉
                        
            except Exception as e:
                self.logger.error(f"Failed at recv_android: {e}")
                raise e
                    
    def send_android(self):
        """
            Send messages to android
        """
        while True:
            try:
                message = self.android_msgs.get_nowait()
                self.android.send(message)
            except queue.Empty:
                # queue is just empty, dont raise error, continue
                continue
        
    def _clear_queues(self):
        """
            Clear both path and command queues
        """
        while not self.commands.empty():
            self.commands.get()

    def _check_api(self):
        """
            Check API connection before starting
        """
        imageRecConnectionResult = self.imageRec.check_connection()
        if not imageRecConnectionResult:
            error_msg = "Image recognition server is down."
            self.logger.error(error_msg)
            self.android_msgs.put(AndroidMessage("error", error_msg))
            raise Exception(error_msg)
        else:
            return
    
    def recv_stm(self):
        """
            Deals with release and aquiring the robot lock
        """
        while True:
            message:str = self.stm.recv()
            try:
                if message.startswith("ACK"):
                    self.movement_lock.release()
            except Exception as e:
                self.logger.error(e)

    def cmd_follower(self):
        while True:
            command: str = self.commands.get()
            try:
                self.movement_lock.acquire()

                # stm32 commands - send to stm
                if command.startswith(STM32_PREFIXES_TASK2):
                    self.stm.send(command)
                    self.logger.debug(f"Sent command to STM: {command}")
                # finish run
                elif command == FIN:
                    self.movement_lock.release()
                    self.logger.info("Finished executing commands.")
                    self.android_msgs.put(AndroidMessage("status", "Finished"))
                    self._stitch()
                else:
                    raise Exception(f"Message of unknown format recieved: {command}")

            except Exception as e:
                self.logger.error("Error in executing commands: {e}")
                self.android_msgs.put(AndroidMessage("error", f"Error occured when executing command :{command}"))
                raise e

    # IMAGE RECOGNITION
    def _snap_and_rec(self):
        """ 
            Snaps a picture and sends it over to the image recognition server 
            to identify image. Returns the image id. 
        """
        image_id = self.imageRec.imageRec("task_2")
        if image_id != 0:
            return image_id
        else:
            self.logger.error("Something wrong happened when requested for snap and rec.")
            pass # image was not recognised
    
    # STITCH IMAGES
    def _stitch(self):
        """
            Run finished. Stitch the image.
             
            TO DO: i think it is better for the stitching to start as 
            soon as the last obstacle is recognised and only display 
            the stitched image on the screen
        """
        
        ret_code = self.imageRec.imageRec("stitch")
        if ret_code == 3:
            self.logger.info("Images stitched!")
            return
        else:
            # handle error
            self.logger.error("Something wrong happened when requested for image stitch.")
            pass 
    
    def stop(self):
        """
            Stop the run
        """
        running_procs = [
            self.proc_android_reciever, 
            self.proc_android_sender, 
            self.proc_cmd_follower, 
            self.proc_stm_reciever
        ]

        for proc in running_procs:
            if proc.is_alive():
                self.logger.info(f"Terminating {proc.name}")
                proc.terminate()
        
        for proc in running_procs:
            proc.join()
            self.logger.info(f"{proc.name} has been stopped.")

        self.logger.info("All processes terminated successfully.")

        # disconnect from all the servers
        self.disconnect()
    
if __name__ == "__main__":
    task2_executor = Task2()
    task2_executor.start()