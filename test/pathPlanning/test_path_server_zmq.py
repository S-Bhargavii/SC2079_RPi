import zmq
import sys
import time
sys.path.append("C:/Users/Bharg/Desktop/SC2079")
sys.path.append("C:/Users/Bharg/Desktop/SC2079/Algorithm")
sys.path.append("C:\\Users\\Bharg\\Desktop\\SC2079\\RPi")

from utility.settings import PATH_API_IP, PATH_API_PORT
from communication.pathPlanning import PathPlanningMessage
from Algorithm.helper import command_generator
from Algorithm.algo.algo import MazeSolver

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind(f"tcp://{PATH_API_IP}:{PATH_API_PORT}")
print("Started server.")
print("Listening to requests")

while True:

    message = socket.recv_json()
    print(f"Recieved: {message}")

    if message["cat"] == "disconnect":
        break

    if message["cat"] == "test":
        print("Client attempting to test connection")
        socket.send_json({}) # note - since this is only a response server, you can not send 2 responses one afetr another

    if message["cat"] == "path":
        # Get the obstacles, big_turn, retrying, robot_x, robot_y, and robot_direction from the json data
        obstacles = message["obstacles"]
        # big_turn = int(content['big_turn'])
        retrying = message['retrying']
        robot_x, robot_y = message['robot_x'], message['robot_y']
        robot_direction = int(message['robot_dir'])
        # Initialize MazeSolver object with robot size of 20x20, bottom left corner of robot at (1,1), facing north, and whether to use a big turn or not.
        maze_solver = MazeSolver(20, 20, robot_x, robot_y, robot_direction, big_turn=None)

        # Add each obstacle into the MazeSolver. Each obstacle is defined by its x,y positions, its direction, and its id
        for ob in obstacles:
            maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['id'])

        start = time.time()
        # Get shortest path
        optimal_path, distance = maze_solver.get_optimal_order_dp(retrying=retrying)
        print(f"Time taken to find shortest path using A* search: {time.time() - start}s")
        print(f"Distance to travel: {distance} units")
        
        # Based on the shortest path, generate commands for the robot
        commands = command_generator(optimal_path, obstacles)

        # Get the starting location and add it to path_results
        path_results = [optimal_path[0].get_dict()]
        # Process each command individually and append the location the robot should be after executing that command to path_results
        i = 0
        for command in commands:
            if command.startswith("SNAP"):
                continue
            if command.startswith("FIN"):
                continue
            elif command.startswith("FW") or command.startswith("FS"):
                i += int(command[2:]) // 10
            elif command.startswith("BW") or command.startswith("BS"):
                i += int(command[2:]) // 10
            else:
                i += 1
            path_results.append(optimal_path[i].get_dict())
        res_message = {
            "distance":distance, 
            "path":path_results, 
            "commands":commands, 
            "error":None
        }
        socket.send_json(res_message)

socket.close()
context.term()

