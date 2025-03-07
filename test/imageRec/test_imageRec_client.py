# test the image recognition module

import sys
sys.path.append("/home/pi/Desktop/RPi")
print(sys.path)
import termios
import tty
from utility.consts import TASK1_IMAGE_IDS


from communication.imageRec import ImageRecLink

print("creating image link")
test_image_rec_link = ImageRecLink()

print("connecting to server")
test_image_rec_link.connect()

# is_active = test_image_rec_link.check_connection()
# test_image_rec_link.set_timeout()

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

# continuous loop
while True:
    key = get_key()  # Read user input
    if key == 'o':  
        image_id = test_image_rec_link.imageRec("task_1")
        # image_str = TASK1_IMAGE_IDS[image_id]
        print(f"Got results of task 1: {image_id}")
    
    elif key == 't':  
        image_id = test_image_rec_link.imageRec("test_task_1")
        image_str = TASK1_IMAGE_IDS[image_id]
        print(f"Got results of task 1: {image_str}")

    elif key == 'q':
        print("Exiting...")
        image_id = test_image_rec_link.imageRec("stitch")
        test_image_rec_link.disconnect()
        break