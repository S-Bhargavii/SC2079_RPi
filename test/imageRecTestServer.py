import imagezmq
import cv2

# Initialize ImageZMQ server
print("Starting ImageZMQ server...")
image_hub = imagezmq.ImageHub()

while True:
    # Receive image from client
    message, image = image_hub.recv_image()

    # Check if message is "close" to stop the server
    if message == "close":
        # perform stitching with the images received
        print("Closing ImageZMQ server...")
        break

    # (Perform image recognition here, replace with actual model)
    label = "1"  # This would be replaced the YOLO model output

    # Send back label
    image_hub.send_reply(label.encode())

    # Show received image (for debugging) and save it
    if cv2.imwrite("image.jpg", image):
        print("Image saved")
