import serial
import time

SERIAL_PORT = "/dev/cu.usbmodem101"
BAUD_RATE = 9600

# measured experimentally
SPEED_MPS = 1  # <-- replace with your measured value

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
time.sleep(2)

def send(vL, vR):
    cmd = f"{vL},{vR}\n"
    ser.write(cmd.encode())
    print("Sending:", cmd.strip())

L = float(input("Enter rectangle length (meters): "))
W = float(input("Enter rectangle width (meters): "))

time_L = L / SPEED_MPS
time_W = W / SPEED_MPS

print("\n=== STARTING RECTANGLE DRY RUN ===")

# Side 1
send(0.4, 0.4)
time.sleep(time_L)
send(0, 0)
input("Rotate rover 90° manually, then press ENTER...")

# Side 2
send(0.4, 0.4)
time.sleep(time_W)
send(0, 0)
input("Rotate rover 90° manually, then press ENTER...")

# Side 3
send(0.4, 0.4)
time.sleep(time_L)
send(0, 0)
input("Rotate rover 90° manually, then press ENTER...")

# Side 4
send(0.4, 0.4)
time.sleep(time_W)
send(0, 0)

print("=== RECTANGLE COMPLETE ===")
ser.close()
