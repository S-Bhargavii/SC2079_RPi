import sys
import tty
import os
import termios

# TODO: 🔹 Set this flag to `True` to enable STM32 communication
USE_STM = False
from communication.stm32 import STMLink
sys.path.append("/home/pi/Desktop/RPi")

def get_input():
    """Reads user input dynamically."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    input_text = ""

    try:
        tty.setraw(fd)  # Set terminal to raw mode
        sys.stdout.write("Enter STM command: ")
        sys.stdout.flush()

        while True:
            char = sys.stdin.read(1)  # Read one character at a time

            # 🔴 Ignore all arrow keys (Up, Down, Left, Right)
            if char == "\x1b":  # Arrow keys start with escape character '\x1b'
                next1 = sys.stdin.read(1)
                next2 = sys.stdin.read(1)
                if next1 == "[" and next2 in "ABCD":  # 'A' = Up, 'B' = Down, 'C' = Right, 'D' = Left
                    continue  # Ignore all arrow keys

            if char in ["\n", "\r"]:  # Handle Enter key (for different terminals)
                break  # Break out of the input loop

            elif char == "\x7f":  # Handle backspace (delete last character)
                if input_text:
                    if input_text[-1] == "|":
                        input_text = input_text[:-2]
                        sys.stdout.write("\b \b\b \b")
                    else:
                        input_text = input_text[:-1]
                        sys.stdout.write("\b \b")
                    sys.stdout.flush()

            # Custom commands
            elif char == "f":  # Replace 'f' with 'T' (forward)
                input_text += "T50|"
                sys.stdout.write("T50|")
                sys.stdout.flush()

            elif char == "b":  # Replace 'b' with 't' (backward)
                input_text += "t50|"
                sys.stdout.write("t50|")
                sys.stdout.flush()

            elif char == "n":  # Replace negative
                input_text += "-"
                sys.stdout.write("-")
                sys.stdout.flush()

            elif char == "p":  # Replace negative
                input_text += "+"
                sys.stdout.write("+")
                sys.stdout.flush()

            elif char == 'q':  # Allow 'q' to exit the input loop
                return None

            else:
                if len(input_text.replace("|", "")) < 9:
                    input_text += char
                    sys.stdout.write(char)
                    sys.stdout.flush()
                    if len(input_text) % 4 == 3 and len(input_text) < 11:  # Insert '|' after every 3 characters
                        input_text += "|"
                        sys.stdout.write("|")
                        sys.stdout.flush()

        return input_text  # Return the collected input text
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Reset terminal settings


# 🔹 Initialize STM connection if flag is enabled
if USE_STM:
    stm_link = STMLink()
    stm_link.connect()

print("🚀 STM32 Command Interface")
print("🔹 Type commands (e.g., T50|-25|87)")
print("🔹 Press 'q' to quit")

while True:
    try:
        command = get_input()
        if command is None:  # If the user pressed 'q' or Ctrl+C
            print("\n🔴 Disconnecting from STM32...")
            if USE_STM:
                stm_link.disconnect()
            break
        elif command.lower() == 'cle|ar':
            os.system('cls' if os.name == 'nt' else 'clear')
        elif command:
            print(f"\n📤 Sending: {command}")
            if USE_STM:
                stm_link.send(command)  # Send command to STM32
            print("✅ Command sent successfully!")
        else:
            print("\n⚠️ Empty command. Try again.")
    except KeyboardInterrupt:
        print("\n🔴 Interrupted. Disconnecting from STM32...")
        if USE_STM:
            stm_link.disconnect()
        sys.exit(0)
    except Exception as e:
        print(f"❌ Error: {e}")
