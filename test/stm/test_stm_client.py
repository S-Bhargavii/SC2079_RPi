import sys
import json
import time 
sys.path.append("/home/pi/Desktop/RPi")

from communication.stm32 import STMLink

# Initialize and connect to STM32
stm_link = STMLink()
stm_link.connect()

print("Enter a command. Type 'q' to quit.")

while True:
    command = input("> ").strip()  # Read full input and remove extra spaces

    if command.lower() == 'q':  # Quit the program
        stm_link.disconnect()
        break
    
    if command:  # If input is not empty
        stm_link.send(command)  
        print(f"Message sent to STM32: {command}")

        # Optionally, receive and print response
        message = stm_link.recv()
        print(f"Message received from STM32: {message}")

print("Disconnected from STM")
