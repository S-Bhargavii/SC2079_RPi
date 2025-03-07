# STM32 BOARD SERIAL CONNECTION
SERIAL_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0002-if00-port0"  # stm32
BAUD_RATE = 115200

HOST_IP_ADDRESS = '192.168.3.20'
# IMAGE API DETAILS
IMAGE_API_IP = HOST_IP_ADDRESS  # IP address of laptop
IMAGE_API_PORT = 5555 # image zmq server port 
IMAGE_API_CHECK_CONNECTION_TIMEOUT = 1000 # no. of milliseconds to wait for test response
IMAGE_API_RESPONSE_TIMEOUT = 3000 # no. of milliseconds to wait for the image api. Set to -1 if want to wait till response comes

# PATH API DETAILS
PATH_API_IP = HOST_IP_ADDRESS
PATH_API_PORT = 5556
PATH_API_CHECK_CONNECTION_TIMEOUT = 500 # no. of milliseconds to wait for test response
PATH_API_RESPONSE_TIMEOUT = 3000 # no. of milliseconds to wait for the path api. Set to -1 if want to wait till response comes

# BLUETOOTH SETTINGS
UUID = "550e8400-e29b-41d4-a716-446655440000" 
