import time
import argparse
from datetime import datetime
from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2
import os
import sys
import sys
import termios
import tty

# create camera object 
camera = PiCamera(resolution=(640, 640))

def get_key():
    """Reads a single keypress from the user."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

id = 0 
image = "ceiling" # change to the directory directory you want to save
file_prefix = f"/home/pi/Desktop/mdp/photos/{image}/"

# continuous loop
while True:
    key = get_key()  # Read user input
    if key == 'o':  
        print("Capturing image")
        id += 1
        camera.capture(f"{file_prefix}/captured_image_{id}.jpg")
    
    elif key == 'q':
        print("Exiting...")
        break

camera.close()