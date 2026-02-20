#Talks to the ESP32 over UART (or CAN/I2C if you’re fancy)
# Sends commands like:
#   V_L, V_R or PWM_L, PWM_R
# Receives feedback like:
#   encoder ticks, motor current, watchdog status

#!/usr/bin/env python3
"""
motor_interface.py
Motor control for 4-wheel rover (front + back)
Python version of your Arduino test
"""

# motor_interface.py
from TerminalMotorInterface import TerminalMotorInterface

def build_motor_interface(config: dict):
    mode = (config.get("motor_mode") or "terminal").lower()

    if mode == "terminal":
        return TerminalMotorInterface(
            turn_gain=config.get("turn_gain", 1.0),
            print_hz=config.get("print_hz", 10),
        )

    if mode == "serial":
        from SerialMotorInterface import SerialMotorInterface
        return SerialMotorInterface(
            port=config.get("serial_port", "/dev/ttyACM0"),
            baud=config.get("serial_baud", 115200),
            timeout=config.get("serial_timeout", 0.02),
        )

    raise ValueError(f"Unknown motor_mode: {mode}")
