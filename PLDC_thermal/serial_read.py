import serial

ser = serial.Serial(
        port='COM8',
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

testnum = input("Test Number:")

f = open(f"thermocouple_data{testnum}.txt", "w")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            f.write(line)
            f.write("\n")
            print(f"Received: {line}")
except KeyboardInterrupt:
    print("Exiting serial read.")
finally:
    ser.close()