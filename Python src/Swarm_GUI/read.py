import serial

# Open serial port
try:
    ser = serial.Serial('COM14', 115200, timeout=1)
    print("Connected to COM14 at 115200 baud.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# Continuously read data
try:
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"{line}")
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()
    print("Serial port closed.")
