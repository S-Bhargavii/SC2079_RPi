import imagezmq
import zmq
import sys
sys.path.append("C:/Users/Bharg/Desktop/SC2079")
sys.path.append("C:/Users/Bharg/Desktop/SC2079/RPi")
import logging
from Image_Recognition.inference.server import ImageRecognitionServer
from utility.settings import IMAGE_API_PORT, IMAGE_API_IP
from utility.logger import configure_logger

# server declaration
SERVER_ADDRESS = f"tcp://{IMAGE_API_IP}:{IMAGE_API_PORT}"

# logger 
logger = logging.getLogger(name="Image_Rec_Server")
configure_logger(logger=logger, linux=False)

# Initialize ImageZMQ server
logger.info("Starting ImageZMQ server....")
image_hub = imagezmq.ImageHub()
yolo_server = ImageRecognitionServer(SERVER_ADDRESS)
image_hub.zmq_socket.setsockopt(zmq.RCVTIMEO, 3000)
ret = "0"

logger.info("Listening to requests..")
try:
    while True:
        try:
            message, image = image_hub.recv_image()
        except zmq.Again:
            continue
        if image is None:
            logger.error("No message recieved.")
            continue 

        if message == "test":
            logger.debug("Checking connection.")
            ret = "1"
        
        elif message == "close":
            logger.info("Closing the imagezmq server.")
            ret = "2"
            image_hub.send_reply(ret.encode())
            break # break from the while loop

        elif message == "checklist":
            prediction_id = yolo_server.run_checklist(image)
            ret = prediction_id

        elif message == "task_1":
            logger.debug("Task 1 image recognition has been requested by the client.")
            # do week 8's image recognition here
            _, prediction_id, _ = yolo_server.run_inference_task_1(image)
            ret = prediction_id # must be the label of the image recognised or 0 if sm error occurred
        
        elif message == "task_2":
            logger.debug("Task 2 image recognition has been requested by the client.")
            # do week 9's image recognition here
            _, prediction_id = yolo_server.run_inference_task_2(image)
            ret = prediction_id # must be the label of the image recognised or 0 if some error occurred

        elif message == "stitch":
            logger.debug("Stitch images has been requested by the client.")
            stitched_image = yolo_server.stitch_images()
            if not stitched_image:
                logger.error("Failed to stitch images..")
            else:
                stitched_image.show()
                ret = "3"
        
        # send back return value
        image_hub.send_reply(ret.encode())

except KeyboardInterrupt:
    logger.info("Ctrl-C pressed, shutting down...")
finally:
    image_hub.close()
    logger.info("Server has been closed")