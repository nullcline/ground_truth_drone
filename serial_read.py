import serial

ser = serial.Serial('COM6', 115200)

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        print(line)

    except KeyboardInterrupt:
        print("\nExiting...")
        ser.close()
        break
