# test the image recognition module

import sys
sys.path.append("/home/pi/Desktop/mdp/code")
print(sys.path)

from communication.imageRec import ImageRecLink

test_image_rec_link = ImageRecLink()
image_id = test_image_rec_link.imageRec()
print(image_id)
test_image_rec_link.cleanup()