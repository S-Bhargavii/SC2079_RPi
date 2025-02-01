import imagezmq
import cv2
import sys
import sys


# Initialize ImageZMQ server
print("Starting ImageZMQ server...")
image_hub = imagezmq.ImageHub()

while True:
    # Receive image from client
    sender_name, image = image_hub.recv_image()
    
    # (Perform image recognition here, replace with actual model)
    label = "1"  

    # Send back label
    image_hub.send_reply(label.encode())

    # Show received image (for debugging)
    if(cv2.imwrite("image.jpg", image)):
        print("Image saved")
        break

