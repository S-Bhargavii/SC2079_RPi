import imagezmq
import time
import random
import sys 
import zmq
sys.path.append("C:/Users/Bharg/Desktop/SC2079")
sys.path.append("C:/Users/Bharg/Desktop/SC2079/RPi")

from Image_Recognition.inference.server import ImageRecognitionServer
from utility.settings import IMAGE_API_PORT, IMAGE_API_IP

# Declaration
SERVER_ADDRESS = f"tcp://{IMAGE_API_IP}:{IMAGE_API_PORT}"

# Initialize ImageZMQ server
print("Starting ImageZMQ server...")
image_hub = imagezmq.ImageHub()
yolo_server = ImageRecognitionServer(SERVER_ADDRESS)
image_hub.zmq_socket.setsockopt(zmq.RCVTIMEO, 1000)
ret = "0"

print("Listening to requests...")

try:
    while True:
        try:
            # Receive image from client
            message, image = image_hub.recv_image()
        except zmq.Again:
            continue
        
        print("recieved msg from client.")
        if image is None:
            print("ERROR: No image received.")

        #  TO DO : change this when I find a better way to test connection
        if message == "test":
            print("testing connection.")
            ret = "1"

        elif message == "close":
            print("Closing imagezmq server...")
            ret = "2"
            image_hub.send_reply(ret.encode())
            break # break away from the while loop
        
        elif message == "task_1":
            print("running task 1 recognition task.")
            # 11 to 35 random int
            time.sleep(5) # sleep to simulate model inference
            rand_id = random.randint(11, 35)
            ret = str(rand_id)
        
        elif message == "task_2":
            print("running task 2 recognition task.")
            # do week 9's image recognition here
            time.sleep(5) # sleep to simulate model inference
            rand_id = random.randint(38, 39)
            ret = str(rand_id)
        
        elif message == "stitch":
            print("stitching images")
            time.sleep(5) # model stitching...
            ret = "3"

        # send back return value
        image_hub.send_reply(ret.encode())

except KeyboardInterrupt:
    print("Ctrl-C pressed, shutting down...")
finally:
    image_hub.close()
    print("Server has been closed")