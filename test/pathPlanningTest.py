# test the image recognition module

import sys
import json
sys.path.append("/home/pi/Desktop/mdp/code")
print(sys.path)

from communication.pathPlanning import PathPlanner

test_path_planner = PathPlanner()
image_id = test_path_planner.connect()
message = {"image_id": image_id, "message": "Hello World!"}
json_msg = json.dumps(message)
test_path_planner.send(json_msg)
test_path_planner.disconnect()