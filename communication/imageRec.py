# code for connecting to the image recognition server
import imagezmq, zmq
import logging
from picamera import PiCamera
from picamera.array import PiRGBArray
from utility.settings import IMAGE_API_PORT, IMAGE_API_IP, IMAGE_API_RESPONSE_TIMEOUT, IMAGE_API_CHECK_CONNECTION_TIMEOUT
from utility.logger import configure_logger

# ImageZMQ is used because it is optimised for fast and efficient messaging of images. It uses tcp protocol for communication.

class ImageRecLink:
    """
        Class to handle image recognition communication
    """
    def __init__(self):
        """
            Just configuring the logger - to keep it same as the other Links
        """
        self.img_count = 0
        self.logger = logging.getLogger("ImageRecLink")
        configure_logger(self.logger)
    
    def connect(self):
        """
            Establishes a connection to the image recognition server
        """
        try:
            address = f"tcp://{IMAGE_API_IP}:{IMAGE_API_PORT}"
            self.image_sender = imagezmq.ImageSender(connect_to=address)
            self.logger.info(f"Connected to Image Recognition Server at {address}")
            self._open_cam() # open the camera after connection to server is successful
        except Exception as e:
            self.logger.error(f"Error in connecting to the image ZMQ server : {e}")
            raise e # raise error to let main thread know

    def check_connection(self):
        """
            Checks it there is an active connection with the image recognition sever

            Returns:
                bool: True if connection is active
        """
        try:
            self.image_sender.zmq_socket.setsockopt(zmq.RCVTIMEO, IMAGE_API_CHECK_CONNECTION_TIMEOUT)
            server_output = self.imageRec("test")
            if(server_output == 1):
                self.logger.info("Connection is active")
                return True
        except Exception as e:
            self.logger.error("Could not connect with the Image recognition server")
            return False
        
        # TO DO : not clean code but can see later
        self.logger.error("Could not connect with the Image recognition server")
        return False

    def set_timeout(self):
        """
            Set socket timeout
        """
        self.image_sender.zmq_socket.setsockopt(zmq.RCVTIMEO, IMAGE_API_RESPONSE_TIMEOUT)

    def imageRec(self, msg:str)->int:
        """
            This is basically the method used to communicate with the server.
            Captures the image and sends it to the image recognition server.
            Args:
                msg: The message that you want to send with the image
            Returns:
                int: code(id number) returned by the server
        """
        server_output = -1
        try:
            # get image from camera
            image = self._take_image()

            # send image to server
            reply = self.image_sender.send_image(msg, image)
            server_output = int(reply.decode("utf-8"))
        except Exception as e:
            self.logger.error(f"Failed to send image to server : {e}")
            server_output = 0 # assigned when something wrong happened  - no meaning to 0
            # raise e # raise the error
        finally:
            self.logger.info(f"Image recognised by server. Image id : {server_output}")
        
        return server_output

    def _take_image(self):
        """
            Takes the image from the rpi
            Returns:
                img: image taken by the rpis
        """
        try:  
            # uncomment the commented code if you want to save 
            # the images to rpi to examine later       

            # capture image
            rawCapture = PiRGBArray(self.camera)
            self.camera.capture(rawCapture, format = "bgr")
            image = rawCapture.array
            rawCapture.truncate(0)
            # self.img_count += 1
            # cv2.imwrite(f"/home/pi/Desktop/RPi/images/captured_image_{self.img_count}.jpg", image)
            self.logger.debug("Captured image")
    
        except Exception as e:
            self.logger.error(f"Failed to take image : {e}")
            # raise e # raise the error
        return image
    
    def _open_cam(self):
        """
            Open the camera
        """
        try:
            self.camera = PiCamera(resolution=(640, 640))
            self.logger.info("Initialised camera")
        except Exception as e:
            self.logger.error(f"Failed to initialise the camera : {e}")
    
    def _close_cam(self):
        """
            Close the camera
        """
        try:
            if self.camera:
                self.camera.close()
                self.logger.info("Closed camera.")
            else:
                self.logger.error("Camera was not initialised.")
        except Exception as e:
            self.logger.error(f"Failed to close the camera : {e}")

    def disconnect(self):
        """
            Clean up after image recongition task is done
        """
        try:
            self.imageRec("close") # last message to close the server
            self.image_sender.close()
            self.logger.info("Image recognition link closed")
            self._close_cam() # close camera after connection to the server is closed
        except Exception as e:
            self.logger.error(f"Failed to close image recognition link : {e}")
            raise e # raise the error