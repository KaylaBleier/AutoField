# also mission_runner.py
#Orchestrates everything
# Starts/stops mission
# Handles states:
#   IDLE → WAIT_FOR_RTK → READY → PAINTING → DONE/ABORT
# Loads config + waypoint file

from motor_interface import build_motor_interface

motors = build_motor_interface(config)
