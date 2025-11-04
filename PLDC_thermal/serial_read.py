import serial
from datetime import datetime

ser = serial.Serial(
        port='COM8',
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

ser2 = serial.Serial(
    port='COM6',
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

testnum = input("Test Number:")

f = open(f"thermocouple_data{testnum}.txt", "w")
f2 = open(f"thermocouple2_data{testnum}.txt", "w")

first1 = 1
first2 = 1

try:
    while True:
        if ser.in_waiting > 0:
            if first1 == 1:
                f.write(f"T_start = {datetime.now()}")
                first1 = 0
            line = ser.readline().decode('utf-8').strip()
            f.write(line)
            f.write("\n")
            print(f"Received: {line}")
        if ser2.in_waiting > 0:
            if first2 == 1:
                f2.write(f"T_start = {datetime.now()}")
                first2 = 0
            line = ser2.readline().decode('utf-8').strip()
            f2.write(line)
            f2.write("\n")
            print(f"Received: {line}")
except KeyboardInterrupt:
    print("Exiting serial read.")
finally:
    ser.close()
    ser2.close()