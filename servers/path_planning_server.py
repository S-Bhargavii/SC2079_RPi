import zmq
import sys
import time
import logging
# chinmays code
sys.path.append("C:/Users/Bharg/Desktop/SC2079")
sys.path.append("C:\\Users\\Bharg\\Desktop\\SC2079\\RPi")

from utility.settings import PATH_API_IP, PATH_API_PORT
from communication.pathPlanning import PathPlanningMessage

from utility.logger import configure_logger
from typing import List

from CAlgorithm.entities.entity import CellState
from CAlgorithm.algorithms.algo import MazeSolver
from CAlgorithm.tools.movement import CommandGenerator
from CAlgorithm.tools.movement import Direction, Motion

# logger 
logger = logging.getLogger(name="Algo_Server")
configure_logger(logger=logger, linux=False)

# server address declartion
SERVER_ADDRESS = f"tcp://{PATH_API_IP}:{PATH_API_PORT}"

# set up socket 
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind(SERVER_ADDRESS)

logger.info("Started server.")
logger.info("Listening to requests")

ROBOT_SIZE_X = 20 
ROBOT_SIZE_Y = 20
STRAIGHT_SPEED = 50
TURN_SPEED = 50
TESTING = False

# chinmays -->
# NORTH = 0 
# SOUTH = 1
# EAST = 2
# WEST = 3
# SKIP = 4

# our old -->
# 0 - North 
# 2 - East
# 4 - South 
# 6 - West

def _get_dict_rep(optimal_path_cell_state: List[CellState]):
    optimal_path_dict = []
    
    for cell_state in optimal_path_cell_state:
        if cell_state.direction == Direction.EAST:
            direction = 2 
        elif cell_state.direction == Direction.NORTH:
            direction = 0 
        elif cell_state.direction == Direction.WEST:
            direction = 6
        else:
            direction = 4

        path = {  # Create a new dictionary inside the loop
            "x": cell_state.x,
            "y": cell_state.y,
            "d": direction
        }
        optimal_path_dict.append(path)

    return optimal_path_dict

try:
    while True:
        try:
            message = socket.recv_json()
        except zmq.Again:
            # Timeout occurred, check if we need to break out of the loop
            continue

        logger.info(f"Received: {message}")

        # use get instead of indexing to avoid errors
        cat = message.get("cat")

        if cat == "disconnect":
            break

        if cat == "test":
            logger.info("Client attempting to test connection")
            socket.send_json({})  # note - since this is only a response server, you can not send 2 responses one after another

        elif cat == "path":
            # Get the obstacles, big_turn, retrying, robot_x, robot_y, and robot_direction from the json data
            obstacles = message.get("obstacles")
            robot_x = message.get('robot_x') 
            robot_y = message.get('robot_y')
            robot_direction = Direction.NORTH

            # Initialize MazeSolver object with robot size of 20x20, bottom left corner of robot at (1,1), facing north, and whether to use a big turn or not.
            maze_solver = MazeSolver(
                size_x=ROBOT_SIZE_X,
                size_y=ROBOT_SIZE_Y,
                robot=None,
                robot_x=robot_x, 
                robot_y=robot_y,
                robot_direction=robot_direction
            )
            command_generator = CommandGenerator(STRAIGHT_SPEED, TURN_SPEED)

            # Add each obstacle into the MazeSolver. Each obstacle is defined by its x,y positions, its direction, and its id
            for ob in obstacles:
                maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['id'])

            # Get shortest path
            start = time.time()
            optimal_path, min_dist = maze_solver.get_optimal_path()
            optimal_path_dict_repr = _get_dict_rep(optimal_path)
            motions, obstacle_ids = maze_solver.optimal_path_to_motion_path(optimal_path)
            end = time.time()

            # Based on the shortest path, generate commands for the robot
            commands = command_generator.generate_commands(motions=motions, 
                                                           obstacle_ids=obstacle_ids, 
                                                           testing=TESTING)
            print(optimal_path)
            res_message = {"commands": commands, "path":optimal_path_dict_repr}
            logger.info(f"Result: {res_message}")
            socket.send_json(res_message)

        else:
            socket.send_json({})

except KeyboardInterrupt:
    logger.info("Ctrl-C pressed, shutting down...")

finally:
    socket.close()
    context.term()
    logger.info("Socket closed.")