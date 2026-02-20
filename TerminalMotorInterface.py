# TerminalMotorInterface.py
import time

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class TerminalMotorInterface:
    """
    Drop-in replacement for SerialMotorInterface.
    Prints commands and the implied left/right wheel signals.
    """
    def __init__(self, turn_gain=1.0, print_hz=10):
        self.turn_gain = float(turn_gain)
        self.print_period = 1.0 / float(print_hz)
        self._last_print = 0.0

    def send_vw(self, v_cmd: int, w_cmd: int, estop: bool = False):
        v_cmd = _clamp(int(v_cmd), -1000, 1000)
        w_cmd = _clamp(int(w_cmd), -1000, 1000)

        # mimic Arduino mixing so you see what wheels would do
        left = int(_clamp(v_cmd - self.turn_gain * w_cmd, -1000, 1000))
        right = int(_clamp(v_cmd + self.turn_gain * w_cmd, -1000, 1000))

        now = time.time()
        if now - self._last_print >= self.print_period:
            self._last_print = now
            if estop:
                print("[MOTOR] ESTOP requested -> v=0 w=0 (left=0 right=0)")
            else:
                print(f"[MOTOR] v={v_cmd:>5} w={w_cmd:>5}  |  left={left:>5} right={right:>5}")

    def stop(self):
        self.send_vw(0, 0, estop=False)

    def estop(self):
        self.send_vw(0, 0, estop=True)
