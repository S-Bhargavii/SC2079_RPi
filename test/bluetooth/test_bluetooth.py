import sys
import json
import termios
import tty
sys.path.append("/home/pi/Desktop/RPi")

from communication.android import AndroidLink, AndroidMessage

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

android_link = AndroidLink()
android_link.connect()

# continuous loop
while True:
    key = get_key()  # Read user input
    if key == "t":
        android_link.send(AndroidMessage("info", "Robot is ready!"))
        message = android_link.recv()
        print(f"message : {message}")

    if key == 'o':  
        android_link.send(AndroidMessage("location", {'robot': {'x': 12, 'y': 7, 'direction': 'W'}, 'command': 'LF090'}))
    
    if key == "r":
        message = android_link.recv()
        print(f"message : {message}")

    elif key == 'q':
        break

android_link.disconnect()