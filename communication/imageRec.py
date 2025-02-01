# code for connecting to the image recognition server
import imagezmq
import logging
from picamera import PiCamera
from picamera.array import PiRGBArray 
from utility.settings import API_PORT, API_IP
from utility.logger import configure_logger

# ImageZMQ is used because it is optimised for fast and efficient messaging of images. It uses tcp protocol for communication.

class ImageRecLink:
    """
        Class to handle image recognition communication
    """
    def __init__(self):
        self.logger = logging.getLogger("ImageRecLink")
        configure_logger(self.logger)
        address = f"tcp://{API_IP}:{API_PORT}"
        self.image_sender = imagezmq.ImageSender(connect_to=address)
        self.logger.info(f"Connected to Image Recognition Server at {address}")
    
    def imageRec(self)->int:
        """
            Captures the image and sends it to the image recognition server.
            Returns the id of the image identified by the server.
        """
        try:
            # open camera
            camera = PiCamera(resolution=(640, 640))
            self.logger.debug("Initialised camera")
        
            # capture image
            rawCapture = PiRGBArray(camera)
            camera.capture(rawCapture, format = "bgr")
            image = rawCapture.array
            rawCapture.truncate(0)
            self.logger.debug("Captured image")

            # send image to server
            reply = self.image_sender.send_image("RaspberryPi", image)
            image_id = int(reply.decode("utf-8"))
        except Exception as e:
            self.logger.error(f"Failed to send image to server : {e}")
            image_id = 0 
            # raise e # raise the error
        finally:
            self.logger.info(f"Image recognised by server. Image id : {image_id}")
            if camera:
                camera.close() # close the camera
                self.logger.debug("Closed camera")
        
        return image_id

    def cleanup(self):
        """
            Clean up after image recongition task is done
        """
        try:
            self.image_sender.close()
            self.logger.info("Image recognition link closed")
        except Exception as e:
            self.logger.error(f"Failed to close image recognition link : {e}")
            # raise e # raise the error

# TO CHECK - 
# 1. Should the camera be initialised in the constructor or should 
# it be initialised in the imageRec method?
        