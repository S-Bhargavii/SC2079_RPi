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
from communication.mock_android import MockAndroidLink
from communication.imageRec import ImageRecLink
from communication.pathPlanning import PathPlannerZMQ, PathPlanningMessage
from communication.stm32 import STMLink
from multiprocessing import Manager, Process
from threading import Thread
from utility.consts import ACKNOWLEDGE, DONE, X, FIRST_OBSTACLE_DISTANCE, SECOND_OBSTACLE_DISTANCE, FIN, SNAP, TASK1_IMAGE_IDS, LEFT_ARROW, RIGHT_ARROW

class Task2:

    def __init__(self):
        """
            Initialises and configures logger
        """
        run_number = 0
        self.logger = logging.getLogger(f"Task2_{str(run_number)}\Task2_{str(run_number)}")
        configure_logger(self.logger)
        self.stm_logger = logging.getLogger(f"Task2_{str(run_number)}\Task2_STM_{str(run_number)}")
        configure_logger(self.stm_logger)
        self.image_rec_logger = logging.getLogger(f"Task2_{str(run_number)}\Task2_Image_Rec_{str(run_number)}")
        configure_logger(self.image_rec_logger)
    
    def create(self):
        """
            Create client objects
        """
        # self.android = MockAndroidLink()
        self.android = AndroidLink()
        self.stm = STMLink()
        self.logger.debug("Created client objects.")
    
    def connect(self):
        """
            Establish connection with servers in parallel.
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
        
        self.imageRec_client_created = True
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

            self.proc_snap_and_rec = Process(target=self.continuous_snap_and_rec, name="image sender")
            self.proc_android_sender = Process(target=self.send_android, name="android sender") # sends messages to android
            self.proc_android_reciever = Process(target=self.recv_android, name="android reciever") # recieve messages from android
            self.proc_cmd_follower = Process(target=self.cmd_follower, name="cmd follower")
            
            # start all processes
            self.proc_snap_and_rec.start()
            self.proc_android_reciever.start()
            self.proc_android_sender.start()
            self.proc_cmd_follower.start()

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
        self.image_msgs = self.manager.Queue
        self.obstacles = self.manager.dict() # can change this to a normal list
        # self.commands = self.manager.Queue()
        self.stm_commands = self.manager.Queue()
        # self.android_commands = self.manager.Queue()
        self.path = self.manager.Queue()
        self.movement_lock = self.manager.Lock()
        self.shared_namespace = self.manager.Namespace()
        # self.shared_namespace.command_executed = ""
        # self.commands_executed = self.manager.Queue()
        self.disconnect_zmq_clients = self.manager.Event()
        self.shared_namespace.move_distance = 0
        self.shared_namespace.first_image_id = -1
        self.shared_namespace.second_image_id = -1

        self.shared_namespace.capturing_first_img = True # true when we are capturing the first image, false when capturing the second image
        self.shared_namespace.continue_capturing = True # true when you want to continue capturing images, false when you want to stop capturing
        self.pathPlanning_client_created = False
    
    # def recv_android(self):
    #     """
    #         Process deals with recieving messages from android

    #         TO DO : don't think a continuous loop is required here 
    #         cuz the android will only send one message through out the run.
    #     """
    #     time.sleep(1)
    #     self.stm_commands.put("START")
    #     self.start_time = time.time()
    #     self.logger.info("Starting robot")

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
                if type == "start" and value == "start":
                    self.stm_commands.put("START")
                    self.start_time = time.time()
                    self.logger.info("Starting robot")
                    self.android_msgs.put(AndroidMessage("status", "Robot started running.")) #🎉              
                else:
                    error_msg = f"An invalid message has been sent by the android : {message}"
                    self.logger.error(error_msg)
                    self.android_msgs.put(AndroidMessage("error", error_msg))
                    raise Exception(error_msg)

            except Exception as e:
                # the individual methods will add error msgs to android queue, 
                # doesnt need to be handled here :)
                self.logger.error(f"Error occured in starting the robot : {e}")

    def continuous_snap_and_rec(self):
        """
            Recieves image code from image recognition server
        """
        # initialise the image recognition server
        self.image_client(connect = True)

        while True:
            try:
                if self.shared_namespace.continue_capturing and self.shared_namespace.capturing_first_img:
                    image_id = self.image_client(snap_and_rec=True, msg = "task_2")
                    if image_id == -1:
                        self.logger.debug("Failed to capture the first image... Retry again...")
                    else:
                        self.shared_namespace.first_image_id = image_id
                        self.logger.info(f"📷 Captured First image : {self.shared_namespace.first_image_id}")
                        self.shared_namespace.continue_capturing = False

                elif self.shared_namespace.continue_capturing and not self.shared_namespace.capturing_first_img:
                    image_id = self.image_client(snap_and_rec=True, msg = "task_2_stitch")
                    if image_id == -1:
                        self.logger.debug("Failed to capture the second image... Retry again...")
                    else:
                        self.shared_namespace.second_image_id = image_id
                        self.logger.info(f"📷 Captured Second image : {self.shared_namespace.second_image_id}")
                        self.shared_namespace.continue_capturing = False
                        break   
                    
            except Exception as e:
                self.logger.error(f"Error occured in starting the robot : {e}")

        self.logger.info("Finished image recognition task. Exited while loop")

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

    def _clear_queues(self):
        """
            Clear both path and command queues
        """
        while not self.stm_commands.empty():
            self.stm_commands.get()

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
        """
            Processes the command string in a way the STM can understand
            by padding only the 2nd and 3rd arguments.
        """
        result = command[:2]  # Extract the command type (e.g., "T")
        arguments = command[2:].split('|')  # Split arguments by '|'

        processed_args = [arguments[0]]  # Keep the first argument unchanged

        if len(arguments) > 1:
            processed_args.append(self._pad_sign(arguments[1]))  # Apply padding to 2nd arg
        if len(arguments) > 2:
            processed_args.append(self._pad_zeros(arguments[2]))  # Apply padding to 3rd arg

        return result + '|'.join(processed_args)
    
    def poll_and_inch(self, direction = None, park = False, first_obs = True) -> int:
        """
            Continuously send INCH.
            If the ACK command is recieved --> the STM is done moving but did not find any obstacle.
            If the DONE command is recieved --> the STM sensed an obstacle nearby

            PARAMS 
            --- 
            direction : int 
                If LEFT_ARROW - interact with the left side Infrared sensor
                If RIGHT_ARROW - interact with the right side Infrared sensor
                If None - interact with the ultrasound sensor
        """
        check = True # just checks without any movement
        inch = 0

        ones_count = 0
        if first_obs:
            safe_dist = 35
        else:
            safe_dist = 35

        while True:
            if check:
                # check from the current distance when first enter this method
                self.stm_logger.info("Checking distance...")
                check = False
                if direction is LEFT_ARROW:
                    command = f"l50|000|000" # stay stationary and check left IR 
                elif direction is RIGHT_ARROW:
                    command = f"r50|000|000" # stay stationary and check right IR
                else:
                    command = f"C50|000|000" # check ultra sound and if distance more than or equal to 50 - DONE... if distance less than 50 - returns distance

            else: 
                # INCH BY 10cm
                self.stm_logger.info("Continue poll and inch")
                if direction is LEFT_ARROW:
                    command = self.process_command(f"L50|000|{10}") # move 10 cm and stay stationary and check left IR 
                elif direction is RIGHT_ARROW:
                    command = self.process_command(f"R50|000|{10}") # move 10 cm and stay stationary and check left IR             
                else:
                    command = self.process_command(f"U50|000|{self.shared_namespace.move_distance}") # move some distance and check the Ultra sound sensor


            self.movement_lock.acquire()
            self.logger.info("Movement lock acquired.")

            self.stm.send(command)
            self.stm_logger.info(f"Sent command to STM: {command}")

            response = self.stm.recv()  # sends distance when it is some distance away, when less than 30 returns done
            self.stm_logger.info(f"Recieved response from STM from poll and inch : {response}")
            
            # Dynamic distance
            if response.isdigit():
                if direction == None and int(response) <= 30:

                    # if park:
                    #     self.logger.debug("Finished")
                    #     self.movement_lock.release()
                    #     return 

                    # only for ultrasound
                    # if (int(response) == safe_dist):
                    #     self.stm_logger.info(f"Prefectly at {safe_dist} units away")

                    dist_to_move_backward = safe_dist - int(response)
                    self.movement_lock.release()
                    cmd_ = self.process_command(f"t50|000|{dist_to_move_backward}")
                    inch -= dist_to_move_backward
                    self._move_robot(cmd_)
                    self.logger.info("😒😒😒😒 Finish inch.")
                    return inch # might need to change inch

                THRESHOLD = 100
                move_dist = int(response) - 30
                if move_dist < THRESHOLD: 
                    distance = move_dist
                else: 
                    distance = THRESHOLD - 30
                
                if command.startswith("R") or command.startswith("L"):
                    self.shared_namespace.move_distance = 10
                    inch += 10
                elif command.startswith("r") or  command.startswith("l"):
                    inch += 0
                else: 
                    if distance == 1:
                        ones_count += 1
                        if ones_count > 2:
                            self.stm_logger.error("⚡⚡ Stuck in a long ultrasonic loop.")
                            distance += 1
                    self.shared_namespace.move_distance = distance
                    inch += self.shared_namespace.move_distance

                self.stm_logger.info(f"Set move distance to: {self.shared_namespace.move_distance}cm")

            if response == DONE:
                self.movement_lock.release()
                self.stm_logger.debug(f"Reached 30cm away, received: {response}")
                self.shared_namespace.move_distance = 0
                # finish polling and inching
                break

            else:
                self.movement_lock.release()
                self.logger.info("Movement lock released.")
                self.logger.debug(f"Inch {self.shared_namespace.move_distance}cm, received: {response}")
                continue      

        self.logger.info("Complete poll and inch. 🎉")
        return inch
    

    def poll_and_inch_park(self, direction = None, park = False, first_obs = True) -> int:
        """
            Continuously send INCH.
            If the ACK command is recieved --> the STM is done moving but did not find any obstacle.
            If the DONE command is recieved --> the STM sensed an obstacle nearby

            PARAMS 
            --- 
            direction : int 
                If LEFT_ARROW - interact with the left side Infrared sensor
                If RIGHT_ARROW - interact with the right side Infrared sensor
                If None - interact with the ultrasound sensor
        """
        check = True # just checks without any movement
        inch = 0
        check_dist = 20
        while True:
            if check:
                # check from the current distance when first enter this method
                self.stm_logger.info("Checking distance...")
                check = False
                command = f"C50|000|000" # check ultra sound and if distance more than or equal to 50 - DONE... if distance less than 50 - returns distance

            else: 
                # INCH BY 10cm
                self.stm_logger.info("Continue poll and inch")
                command = self.process_command(f"U50|000|{self.shared_namespace.move_distance}") # move some distance and check the Ultra sound sensor


            self.movement_lock.acquire()
            self.logger.info("Movement lock acquired.")

            self.stm.send(command)
            self.stm_logger.info(f"Sent command to STM: {command}")

            response = self.stm.recv()  # sends distance when it is some distance away, when less than 30 returns done
            self.stm_logger.info(f"Recieved response from STM from poll and inch : {response}")
            
            # Dynamic distance
            if response.isdigit():
                if direction == None and int(response) <= check_dist:
                    # stop if the distance is less than 20
                    self.logger.debug("Finished")
                    self.movement_lock.release()
                    return

                # THRESHOLD = 40
                # move_dist = int(response) - check_dist
                # if move_dist < THRESHOLD: 
                #     distance = move_dist
                # else: 
                #     distance = THRESHOLD - check_dist

                max_movement = 20
                move_dist = int(response) - check_dist
                distance = min(move_dist, max_movement)

                # if distance == 1:
                #     ones_count += 1
                #     if ones_count > 2:
                #         self.stm_logger.error("⚡⚡ Stuck in a long ultrasonic loop.")
                #         distance += 1

                if distance <= 2:
                    return inch

                self.stm_logger.debug(f"🚘🚘 The car moved : {distance} while parking.")

                self.shared_namespace.move_distance = distance
                # inch += self.shared_namespace.move_distance

                self.stm_logger.info(f"Set move distance to: {self.shared_namespace.move_distance}cm")

            if response == DONE:
                self.movement_lock.release()
                self.stm_logger.debug(f"Reached 30cm away, received: {response}")
                self.shared_namespace.move_distance = 0
                # finish polling and inching
                break

            else:
                self.movement_lock.release()
                self.logger.info("Movement lock released.")
                self.logger.debug(f"Inch {self.shared_namespace.move_distance}cm, received: {response}")
                continue      

        self.logger.info("Complete poll and inch. 🎉")
        return inch

    def big_turn(self, is_right = False, micro_turn = False):
        """
            This method is to be called to move around the first obstacle.

            Params
            ---
            is_right : bool
                true if the arrow on the first obstacle is right.

            micro_turn : bool
                true if want to finish movement in 3 commands else false.
        """
        if micro_turn:

            if is_right:
                # self._move_robot("T50|+80|038")
                # self._move_robot("T50|-55|075")
                # self._move_robot("T50|+80|030")

                # V2
                # self._move_robot("T50|+80|040")
                # self._move_robot("T50|-55|070")
                # self._move_robot("T50|+80|030")

                # self._move_robot("T50|+80|042")
                # self._move_robot("T50|-55|085")
                # self._move_robot("T50|+80|040")

                # self._move_robot("T50|+80|045")
                # self._move_robot("T50|-55|085")
                # self._move_robot("T50|+80|040")

                self._move_robot("T50|+80|050")
                self._move_robot("T50|-55|100")
                self._move_robot("T50|+80|050")
                return 
            else:
                # self._move_robot("T50|-55|042")
                # self._move_robot("T50|+80|075")
                # self._move_robot("T50|-55|030")

                # V2
                # self._move_robot("T50|-55|042")
                # self._move_robot("T50|+80|080")
                # self._move_robot("T50|-55|032")

                # self._move_robot("T50|-55|042")
                # self._move_robot("T50|+80|080")
                # self._move_robot("T50|-55|032")

                # self._move_robot("T50|-55|047")
                # self._move_robot("T50|+80|090")
                # self._move_robot("T50|-55|040")

                self._move_robot("T50|-55|050")
                self._move_robot("T50|+80|100")
                self._move_robot("T50|-55|050")
                return
            
        else:
            if is_right:
                self.turn(RIGHT_ARROW)
                self.turn(LEFT_ARROW)
                self.turn(LEFT_ARROW)
                self.turn(RIGHT_ARROW)
            else:
                self.turn(LEFT_ARROW)
                self.turn(RIGHT_ARROW)
                self.turn(RIGHT_ARROW)
                self.turn(LEFT_ARROW)

        self.move_forward_straight(0)
    
    def turn_180(self, direction):
        """
            Commands the robot to make a 180 degree turn.
            Whether to make a left 180 degree turn or a right 180 degree turn will 
            depend on the direction of the arrow on the second obstacle.

            Params
            --
            direction : str
                The direction of the second obstacle recognised by the image rec server.
        """
        try:
            if direction is LEFT_ARROW:
                command = "T50|+87|180"
            else:
                command = f"T50|-50|180"

            self._move_robot(command)
            self.move_forward_straight(0)
            self.logger.info("Turned 180 degrees")

        except Exception as e:
            self.logger.error(f"Failed during 180 turn: {str(e)}", exc_info=1)
    
    def turn(self, direction):
        """
            Executes a normal left or right turn based on image output.

            Params
            ---
            direction : int 
                The direction on the obstacle acc to which we will turn the robot.
        """
        try:
            # check if the direction is a valid direction
            if direction not in (LEFT_ARROW, RIGHT_ARROW):
                # should never happen
                self.logger.error(f"Invalid turn direction : {direction}.")
                return 

            if direction is LEFT_ARROW:
                self._move_robot("T50|-60|089") # original
                # self._move_robot("T50|-75|089")
                # self._move_robot("T50|000|008")
            else:
                self._move_robot("T50|+80|089") # original
                # self._move_robot("T50|+80|089")
                # self._move_robot("T50|000|005")

            self.move_forward_straight(0)
            self.logger.info(f"Executed TURN_{direction}")

        except Exception as e:
            print(f"Error when turning: {e}")

    
    def opposite_turn(self, direction):
        """ 
            Return the opposite of a given turn direction ('L' -> 'R', 'R' -> 'L'). 

            Params
            --
            direction : int 
                The direction that you want to reverse.
            
            Returns 
            --
            int 
                The direction opposite to the parameter that you passed.
        """
        if direction == RIGHT_ARROW:
            return LEFT_ARROW
        else:
            return RIGHT_ARROW

    def move_forward_straight(self, distance):
        """
            Move straight forward by a given distance. Internally calls 
            the _move_robot command after constructing the command
            using process_command

            Params
            --
            distance : int 
                Amount of distance to be moved forward
        """
        try:
            self.logger.info(f"Moving by {str(distance)}")
            # process the command
            command = self.process_command(f"T50|000|{str(distance)}")
            self._move_robot(command)

        except Exception as e:
            self.logger.error(f"Something went wrong with the move method : {e}")
    
    def _move_robot(self, command):
        """
            This method is used to send a command to the robot.
            1. It acquires the robot lock.
            2. It sends the command to the robot.
            3. It waits for the acknowledgment message from the robot and then releases the lock.

            Params
            ---
            command : str
                The exact command that you want to send to the robot.
        """
        try:
            self.movement_lock.acquire()
            self.logger.debug("Movement lock acquired.")
            self.stm.send(command)
            self.stm_logger.info(f"Sent command to STM: {command}")

            response = self.stm.recv()
            self.stm_logger.debug(f"Recieved response from STM : {response}")

            if response is not DONE:
                self.movement_lock.release()
                self.logger.debug("Movement lock released.")
            else:
                raise Exception("Did not recive acknowledgment message from the robot!! 💀💀")
            
        except Exception as e:
            self.logger.error(f"Something went wrong in moving the robot : {e}")
            raise e
    
    def _retrieve_image_id(self, number: int):
        """
            Returns the image id of the obstacle number provided.
        """
        if number == 1:
            count = 0 
            while self.shared_namespace.first_image_id == -1 and count < 4:
                self.logger.debug(f"Waiting {0.5*count}s")
                if self.shared_namespace.first_image_id != -1:
                    # found image id break
                    break
                
                time.sleep(0.5)
                count += 1
            
            if self.shared_namespace.first_image_id == -1:
                self.logger.info("using default left arrow for first obstacle.")
                return LEFT_ARROW
            return self.shared_namespace.first_image_id

        elif number == 2:
            count = 0 
            while self.shared_namespace.second_image_id == -1 and count < 4:
                self.logger.debug(f"Waiting {0.5*count}s")
                if self.shared_namespace.second_image_id != -1:
                    # found image id break
                    break
                
                time.sleep(0.5)
                count += 1
            
            if self.shared_namespace.second_image_id == -1:
                self.logger.info("using default left arrow for first obstacle.")
                return LEFT_ARROW
            return self.shared_namespace.second_image_id

    def cmd_follower(self):
        # while loop to wait for the start command
        while True:
            try:
                command = self.stm_commands.get_nowait()
            except queue.Empty:
                continue
            if command == "START":
                self.logger.info("🍜 STEP 0: START COOKING")
                # start logic once START cmd is recieved
                break

        try:
            a, b, y, z = 0, 0, 0, 0
            image_id_1, image_id_2 = -1, -1

            self.start_time = time.time()
            # 1. Move by 30
    
            # self.logger.info("🍜 STEP 1: MOVE BY 30")
            # self.move(FIRST_OBSTACLE_DISTANCE)

            # 2. Send INCH commands until DONE is received
            self.logger.info("🍜 STEP 1: POLLING AND INCHING - move to first obstacle.")
            self.stm_logger.info("🍜 STEP 1: POLLING AND INCHING - move to first obstacle.")
            a += self.poll_and_inch(first_obs = True)
            self.logger.info(f"Recorded A: {a}")

            image_id_1 = self._retrieve_image_id(1)
            
            # stop capturing until pass the first obstacle
            self.shared_namespace.continue_capturing = False

            # ADJUSTMENT TO MAKE UP FOR STM 50cm THRESHOLD
            # self.move(10)


            # 4. Make BIG_TURN with micro turn
            self.logger.info("🍜 STEP 2: MAKE TURN AROUND FIRST OBSTACLE.")
            is_right = image_id_1 == RIGHT_ARROW

            
            # this is the turn to go around the first obstacle
            self.big_turn(micro_turn = True, is_right = is_right)

            self.shared_namespace.capturing_first_img = False # signal to start capturing second image
            self.shared_namespace.continue_capturing = True # continue capturing

            # 5. Move by (60 - X)
            # TO DO - WHY WE DOING THIS
            # if SECOND_OBSTACLE_DISTANCE - X:
            #     self.logger.info("🍜 STEP 3: MOVE BY (60-X)")
            #     self.move_forward_straight(SECOND_OBSTACLE_DISTANCE - X)
            
            # 6. Send INCH commands until DONE is received
            self.logger.info("🍜 STEP 4: POLLING AND INCHING - move to second obstacle")
            self.stm_logger.info("🍜 STEP 2: POLLING AND INCHING - move to second obstacle")
            b += self.poll_and_inch(first_obs = False)
            self.logger.info(f"Recorded B: {b}")

            self.shared_namespace.continue_capturing = False  # stop capturing

            # give 2s to check for image
            image_id_2 = self._retrieve_image_id(2)

            # # TODO: REVERSE FOR SAFETY (Add)
            # if True:
            #     self.logger.info("Reverse for safety")
            #     self._move_robot("t50|000|030")

            # Reverse for safety
            # self._move_robot("t50|000|020")
            
            self.logger.info("🍜 STEP 5: TURN TO BEGIN TRAVERSING ALONG THE LONG OBSTACLE.")
            self.turn(image_id_2)
            
            if image_id_2 == RIGHT_ARROW:
                self._move_robot("t50|000|010")

            # 8. Send INCH commands until DONE is received
            self.logger.info("🍜 STEP 6: POLLING AND INCHING - moving along the long obstacle inside")
            self.stm_logger.info("🍜 STEP 3: POLLING AND INCHING - moving along the long obstacle inside")
            y += self.poll_and_inch(image_id_2)
            self.logger.info(f"Recorded y: {y}")

            # 9. Make 180 degree turn around the obstacle
            self.logger.info("🍜 STEP 7: MAKING 180 degree TURN")
            self.turn_180(image_id_2)

            # 10. Move by (y + 30 + 15)
            self.logger.info("🍜 STEP 8: MOVE BY Y+30+15")
            # self.move_forward_straight(30 + y)
            if image_id_2 == RIGHT_ARROW: 
                y += 56 
            else:
                y += 50 

            y -= 15
            self.move_forward_straight(y)

            # 11. Send INCH commands until DONE is received
            self.logger.info("🍜 STEP 9: POLLING AND INCHING - moving along the long obstacle outside")
            self.stm_logger.info("🍜 STEP 4: POLLING AND INCHING - moving along the long obstacle outside")
            z += self.poll_and_inch(image_id_2)

            # 12. Make opposite of 2nd arrow
            self.logger.info("🍜 STEP 10: MAKE OPPOSITE OF 2ND ARROW")
            self.turn(self.opposite_turn(image_id_2))

            # 13. Move by CALC
            # total_distance = a + b + 40 + 60 + 20 - 30 - 30  #  20cm for 2nd, 

            # distance_from_second_obstacle_to_start_of_the_carpark = ( a + 35 ) + ( b + 35 ) + 10  --> THIS WAS THE VALUE BEFORE
            # total_distance = distance_from_second_obstacle_to_start_of_the_carpark + 5 --> THIS ALSO WE DID BEFORE

            distance_from_second_obstacle_to_start_of_the_carpark = ( a + 35 ) + ( b + 35 ) # +10
            
            # if image_id_1 == RIGHT_ARROW :
            #     distance_from_second_obstacle_to_start_of_the_carpark += 56
            # else:
            #     distance_from_second_obstacle_to_start_of_the_carpark += 55
            
            total_distance = distance_from_second_obstacle_to_start_of_the_carpark + 15 # was 20 before 
            self.logger.info(f"😶‍🌫️distance_from_second_obstacle_to_start_of_the_carpark : {distance_from_second_obstacle_to_start_of_the_carpark}")
            self.logger.info(f"😶‍🌫️total_distance :  {total_distance}")
            # how much did the robots start point move from the inner edge of the obstacle --> subtract that 
            # how much distance does it move to turn inside the carpark --> subtract that

            self.logger.info(f"🍜 STEP 11: MOVE STRAIGHT BY TOTAL DISTANCE: {total_distance}")
            # total_distance -= 10
            self.move_forward_straight(total_distance)

            # STOP AFTER THE FIRST OBSTACLE TO MAKE TURN

            # 14. Make opposite of 2nd arrow
            self.logger.info("🍜 STEP 12: MAKE OPPOSITE OF 2ND ARROW")
            self.turn(self.opposite_turn(image_id_2))

            # REVERSE FOR SAFETY
            # self._move_robot("t50|000|010") ----> THIS WE DID JUST NOW
            
            # z -= 10 ----> THIS WE DID JUST NOW
            self.logger.info(f"Recorded z: {z}")

            if image_id_2 == RIGHT_ARROW:
                y -= 24
                # it was 20 before
            else:
                y -= 20

            z += y
            self.logger.info(f"🥸🥸🥸🥸distance of the long obstacle : {z}🥸🥸🥸🥸")
            z = (z//2) 

            # self.move_forward_straight(y)
            self.move_forward_straight(z)
            
            # 14. Move by Z
            # self.logger.info(f"🍜 STEP 13: MOVE BY Z: {z}")
            # if z > 30: 
            #     self.move_forward_straight(z)

            # 15. Make final TURN
            self.logger.info("🍜 STEP 14: MAKE FINAL TURN TO ALIGN || TO CARPARK")
            self.turn(image_id_2)

            # 16. PARK THE CAR
            self.logger.info("🍜 STEP 15: PARK THE CAR")
            self.poll_and_inch_park(park = True)
            
            self.end_time = time.time()
            self.logger.info(f"🏁🏁🏁 Run time : {self.end_time - self.start_time} 🏁🏁🏁")
            
            # 17. SEND FINISH MESSAGE TO ANDROID
            self.logger.info("YIPPEEEEEEE. 🎉🎉🎉🎉🎉")
            self.android_msgs.put(AndroidMessage("status", "Finished")) #🎉  

        except Exception as e:
            self.logger.error(f"Error in executing command sequence: {e}")
            self.android_msgs.put(AndroidMessage("error", f"Error occurred: {e}"))
            raise e

    def image_client(self, snap_and_rec = True, disconnect = False, connect=False, msg = "task_2"):
        if connect:
            self.imageRec = ImageRecLink()
            self.imageRec.connect()
            return 

        if disconnect:
            self.logger.debug("Disconnecting from imagezmq server.")
            self.imageRec.disconnect()
            return
        
        if snap_and_rec:
            self.logger.debug("Snap and Rec requested to the image zmq server.")
            image_id = self.imageRec.imageRec(msg)
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
            self.proc_snap_and_rec
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
    task1_executor = Task2()
    task1_executor.start()