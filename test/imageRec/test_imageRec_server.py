import imagezmq
import cv2

import sys 
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
ret = "0"

while True:
    print("Listening to requests...")
    # Receive image from client
    message, image = image_hub.recv_image()

    if image is None:
        print("ERROR: No image received.")

    #  TO DO : change this when I find a better way to test connection
    if message == "test":
        ret = "1"

    elif message == "close":
        print("Closing imagezmq server...")
        ret = "2"
        image_hub.send_reply(ret.encode())
        image_hub.close()
        break # break away from the while loop

    elif message == "checklist":
        yolo_server.run_checklist(image)


    elif message == "test_task_1":
        # do week 8's image recognition here
        prediction_id = yolo_server.run_test(image)
        ret = prediction_id # must be the label of the image recognised or 0 if sm error occurred
    
    elif message == "task_1":
        # do week 8's image recognition here
        _, prediction_id, _ = yolo_server.run_inference_task_1(image)
        ret = prediction_id # must be the label of the image recognised or 0 if sm error occurred
    
    elif message == "task_2":
        # do week 9's image recognition here
        _, prediction_id = yolo_server.run_inference_task_2(image)
        ret = prediction_id # must be the label of the image recognised or 0 if some error occurred
    
    elif message == "stitch":
        stitched_image = yolo_server.stitch_images()

        if not stitched_image:
            print("Failed to stitch images.")
        else:
            stitched_image.show()
            ret = "3"

    # send back return value
    image_hub.send_reply(ret.encode())


print("Server has been closed")


# TO TEST ONLY THE RPi CLIENT laptop COMMUNICATION W/O THE IMAGE SERVER

# import imagezmq
# import cv2

# # Initialize ImageZMQ server
# print("Starting ImageZMQ server...")
# image_hub = imagezmq.ImageHub()

# while True:
#     # Receive image from client
#     message, image = image_hub.recv_image()

#     if message == "close":
#         print("Closing imagezmq server...")
#         ret = "2" # return value
#         image_hub.send_reply(ret.encode())
#         image_hub.close()
#         break # break away from the while loop
    
#     elif message == "test":
#         ret = "1"
    
#     elif message == "task_1":
#         # do week 8's image recognition here
#         ret = "3" # must be the label of the image recognised or 0 if sm error occured
    
#     elif message == "task_2":
#         # do week 0's image recognition here
#         ret = "4"  # must be the label of the image recognised or 0 if some error occured
    
#     elif message == "stitch":
#         pass # begin stitching

#     else:
#         ret = "0" # some error occured
    
#     # send back return value
#     image_hub.send_reply(ret.encode())

#     # Show received image (for debugging) and save it
#     if cv2.imwrite("image.jpg", image):
#         print("Image saved")

# print("Server has been closed")