import sys
import json
sys.path.append("/home/pi/Desktop/RPi")
print(sys.path)

from communication.pathPlanning import PathPlannerZMQ, PathPlanningMessage

test_path_planner = PathPlannerZMQ()

test_path_planner.connect()

is_active = True

if is_active:
    print("server is active")
    path_plan_msg = PathPlanningMessage("path", {
        "obstacles":[
            {
                "x":18,
                "y":9, 
                "id":1,
                "d":3
            },
            {
                "x":5,
                "y":9,
                "id":2,
                "d":1
            },
            {
                "x":18,
                "y":17,
                "id":3,
                "d":3
            },
            {
                "x":7,
                "y":13,
                "id":4,
                "d":0
            }
        ],
        "robot_x": 1,
        "robot_y": 1, 
        "robot_dir": 0
    })
    
    test_path_planner.send(path_plan_msg)

    rec_message = test_path_planner.recv()
    test_path_planner.disconnect()
else:
    print("Connection to path planner is not established.")
