import evdev
from evdev import InputDevice, categorize, ecodes
import serial
import serial.tools.list_ports
import time
import json

def send_data(ser, buttons, leftX, leftY, rightX, rightY):
    output = f"Sending data: buttons={buttons}, leftX={leftX}, leftY={leftY}, rightX={rightX}, rightY={rightY}"
    if output != "Sending data: buttons=0, leftX=127, leftY=127, rightX=127, rightY=127":
        print(f"Sending data: buttons={buttons}, leftX={leftX}, leftY={leftY}, rightX={rightX}, rightY={rightY}")
    buffer = bytearray(7)  # Increase buffer size to 7 to include checksum
    buffer[0] = buttons & 0xFF  # 按鈕低8位
    buffer[1] = (buttons >> 8) & 0x3F  # 按鈕高6位（只用14個按鍵）
    buffer[2] = leftX
    buffer[3] = leftY
    buffer[4] = rightX
    buffer[5] = rightY
    buffer[6] = sum(buffer[:6]) & 0xFF  # Calculate checksum as the sum of the first 6 bytes modulo 256
    ser.write(buffer)

def normalize(value, min_input, max_input, min_output, max_output):
    """Normalize a value from one range to another."""
    value = max(min(value, max_input), min_input)  # Clamp the value to the input range
    return int((value - min_input) * (max_output - min_output) / (max_input - min_input) + min_output)

def list_USB_devices():
    # 列出所有可用的串列埠
    ports = serial.tools.list_ports.comports()

    # 過濾出 /dev/ttyACM 裝置
    for port in ports:
        if "ttyS" not in port.device:
            print(f"裝置: {port.device}")
            print(f"描述: {port.description}")
            print(f"製造商: {port.manufacturer}")
            print(f"產品ID: {port.pid}, 廠商ID: {port.vid}")
            print(f"序號: {port.serial_number}")
            print("-" * 40)

def main():
    # List all input devices
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    print("Available devices:")
    for device in devices:
        print(f"{device.path}: {device.name}")

    # Prompt user to select the Joy-Con device
    device_path = f"/dev/input/event{input('Enter the event number for your Joy-Con (e.g., /dev/input/eventX): ')}"
    try:
        joycon = InputDevice(device_path)
        print(f"Connected to {joycon.name}")
    except FileNotFoundError:
        print("Device not found. Please check the path and try again.")
        return

    # Load button mapping from JSON file
    if "L" in joycon.name:
        print("Left Joy-Con detected. Loading left button mapping.")
        button_mapping = json.load(open("key_mapping_L.json", "r"))
    else:
        print("Right Joy-Con detected. Loading right button mapping.")
        button_mapping = json.load(open("key_mapping_R.json", "r"))
    print(button_mapping)

    # Prompt user to select the COM port
    list_USB_devices()
    COM_port = input("Enter your COM port (e.g., COM3 or /dev/ttyACM0): ")
    if not COM_port:
        COM_port = "/dev/ttyACM0" # Default for Arduino on Linux
    try:
        ser = serial.Serial(COM_port, 115200)
        print(f"Connected to Arduino on {COM_port}")
    except serial.SerialException:
        print("Failed to connect to the COM port. Please check and try again.")
        return

    print("Listening for Joy-Con input events. Press Ctrl+C to exit.")
    try:
        buttons = 0
        leftX = 127
        leftY = 127
        rightX = 127
        rightY = 127

        for event in joycon.read_loop():
            if event.type == ecodes.EV_KEY:  # Button events
                if event.value == 1:  # Button pressed
                    print(f"Button pressed: {str(categorize(event).keycode)}")
                    if str(categorize(event).keycode) in button_mapping:
                        print(f'sent {button_mapping[str(categorize(event).keycode)]}')
                        buttons |= (1 << button_mapping[str(categorize(event).keycode)])
                elif event.value == 0:  # Button released
                    if str(categorize(event).keycode) in button_mapping:
                        buttons &= ~(1 << button_mapping[str(categorize(event).keycode)])
            elif event.type == ecodes.EV_ABS:  # Joystick events
                if categorize(event).event.code == ecodes.ABS_X:  # Left stick X-axis
                    leftY = normalize(-(event.value), joycon.absinfo(ecodes.ABS_X).min, joycon.absinfo(ecodes.ABS_X).max, 0, 255)  # Adjust input range
                elif categorize(event).event.code == ecodes.ABS_Y:  # Left stick Y-axis
                    leftX = normalize(event.value, joycon.absinfo(ecodes.ABS_Y).min, joycon.absinfo(ecodes.ABS_Y).max, 0, 255)  # Adjust input range
                elif categorize(event).event.code == ecodes.ABS_RX:  # Right stick X-axis
                    leftY = normalize(event.value, joycon.absinfo(ecodes.ABS_RX).min, joycon.absinfo(ecodes.ABS_RX).max, 0, 255)  # Adjust input range
                elif categorize(event).event.code == ecodes.ABS_RY:  # Right stick Y-axis
                    leftX = normalize(-(event.value), joycon.absinfo(ecodes.ABS_RY).min, joycon.absinfo(ecodes.ABS_RY).max, 0, 255)  # Adjust input range

            send_data(ser, buttons, leftX, leftY, rightX, rightY)

            if ser.in_waiting > 0:
                line = ser.readline()
                print(f"Arduino says: {line.decode().strip()}")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
