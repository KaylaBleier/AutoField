#!/usr/bin/env python3
"""
AutoField Dry Run v3 - MAJOR POINTS ONLY
"""

import math
import time
import sys

class MotorControl:
    def __init__(self):
        self.vL = 0.0
        self.vR = 0.0  
        self.spray = False
        
    def set_velocity(self, vL: float, vR: float):
        self.vL, self.vR = max(-1.0, min(1.0, vL)), max(-1.0, min(1.0, vR))
        # NO PRINT - silent
    
    def set_spray(self, paint: bool):
        self.spray = paint
        # NO PRINT - silent

class SimPosition:
    def __init__(self):
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.vL, self.vR = 0.0, 0.0
        
    def update(self, motors: MotorControl, dt: float, verbose: bool = False):
        self.vL, self.vR = motors.vL, motors.vR
        v_fwd = (self.vL + self.vR) / 2
        omega = (self.vR - self.vL) / 0.4
        self.x += v_fwd * math.cos(self.theta) * dt
        self.y += v_fwd * math.sin(self.theta) * dt  
        self.theta += omega * dt
        
        if verbose:
            print(f"📍 ({self.x:.1f}m, {self.y:.1f}m) θ={math.degrees(self.theta):.0f}°")

def execute_straight(L: float, paint: bool, motors: MotorControl, pos: SimPosition, speed=0.5):
    print(f"\n➡️  START: Straight {abs(L):.0f}m {'PAINT' if paint else ''}")
    print(f"📍 START: ({pos.x:.1f}m, {pos.y:.1f}m)")
    
    motors.set_spray(paint)
    direction = 1 if L >= 0 else -1
    distance = abs(L)
    
    start_x, start_y = pos.x, pos.y
    
    # FAST SIMULATION + PROGRESS
    steps = int(distance / 5) + 1  # 5m steps
    for i in range(steps):
        dt = distance / steps  # Even steps
        motors.set_velocity(speed * direction, speed * direction)
        pos.update(motors, dt)
        if i % 5 == 0:  # Every 25m
            print(f"   {int((i/steps)*100)}% ({pos.x:.0f}m)")
        time.sleep(0.01)  # 100x faster!
    
    motors.set_velocity(0, 0)
    print(f"✅ END: ({pos.x:.1f}m, {pos.y:.1f}m)")


def execute_turn_90_left(radius: float, motors: MotorControl, pos: SimPosition):
    """270° turn - PRINT START/END only"""
    print(f"\n🔄 START: 270° LEFT r={radius}m")
    print(f"📍 START: ({pos.x:.1f}m, {pos.y:.1f}m)")
    
    motors.set_spray(False)
    cx, cy = pos.x + radius, pos.y + radius
    arc_angle = 1.5 * math.pi
    
    start_x, start_y = pos.x, pos.y
    
    steps = int(arc_angle * radius / 0.1)
    for i in range(steps):
        angle = arc_angle * (i + 1) / steps
        target_x = cx - radius * math.cos(angle)
        target_y = cy - radius * math.sin(angle)
        
        heading = math.atan2(target_y - pos.y, target_x - pos.x)
        v_fwd = 0.3
        vL = v_fwd - 0.8 * heading * 0.4
        vR = v_fwd + 0.8 * heading * 0.4
        
        motors.set_velocity(vL, vR)
        pos.update(motors, 0.05)
    
    motors.set_velocity(0, 0)
    print(f"✅ END: ({pos.x:.1f}m, {pos.y:.1f}m)")

def main():
    length_yd, width_yd = 100, 70
    length, width = length_yd * 0.9144, width_yd * 0.9144
    turn_radius = 1.0
    
    print(f"⚽ Soccer field: {length_yd}×{width_yd}yd = {length:.1f}×{width:.1f}m")
    print(f"🔄 Turn radius: {turn_radius}m")
    print("=" * 50)
    
    motors = MotorControl()
    pos = SimPosition()
    
    try:
        # YOUR EXACT SEQUENCE - CLEAN OUTPUT
        execute_straight(length, True, motors, pos)      # bottom
        execute_turn_90_left(turn_radius, motors, pos)   # turn 1
        execute_straight(width, True, motors, pos)       # left  
        execute_turn_90_left(turn_radius, motors, pos)   # turn 2
        execute_straight(-length, True, motors, pos)     # top
        execute_turn_90_left(turn_radius, motors, pos)   # turn 3
        execute_straight(-width, True, motors, pos)      # right
        
        print("\n🎉 FULL PERIMETER COMPLETE!")
        print(f"🏁 FINAL: ({pos.x:.1f}m, {pos.y:.1f}m)")
        
    except KeyboardInterrupt:
        print("\n🛑 STOPPED")
    finally:
        motors.set_velocity(0, 0)
        motors.set_spray(False)

if __name__ == "__main__":
    main()
