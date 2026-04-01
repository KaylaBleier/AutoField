# Autofield Rover — Test Day
This is a laptop-based run — once we confirm it working here we move to the Pi.
---

## Setup
Download `autofield.3` and open a terminal inside it. Pull the Arduino files out and move them into your Arduino sketchbook (`Documents/Arduino/rover_combined/rover_control_4.ino`).
---

## 1. Load the Arduino sketch
> ⚠️ Motors off for this step.

Open `rover_coontrol_4.ino` in the Arduino IDE, select **Tools → Board → Arduino Uno**, pick the right port, and upload.

---

## 2. Find your COM ports
Open Device Manager and note the port for each device when plugged in:

| Device | Expected port | Baud |
|---|---|---|
| GPS receiver | COM5 | 57600 |
| Arduino | COM15 | 9600 |

If yours differ, update `GNSS_PORT` in `main.py` and `SERIAL_PORT` in `path_following.py` before running.

---

## 3. Full run
Power everything on, give the GPS a minute outside to get a fix, then:

```
py main.py
```

The script will prompt you through the rest — orient the rover, press Enter to record the start position, enter a line length, press Enter to go. Put the laptop on the rover and let it run.

**To stop at any point:** `Ctrl+C`

**If it doesn't stop at the destination:** unplug the Arduino — the onboard watchdog cuts the motors within 0.5 s.

---

## 4. Quick test
For a fast check without the full startup sequence:

```
py test_dummy_rover_2.py
```

Choose **option 3**. The ports and line length are pre-filled — just press Enter through the prompts. If the Arduino fails to connect it'll warn you and keep running in print-only mode so you can still check the GPS and path logic.

All runs log to a timestamped CSV in `track_logs/`. Don't worry, the data is saved. Just upload that folder to the drive after all testing is done. 

---

## Troubleshooting
**"Cannot open GPS port"** — wrong COM number. Update `GNSS_PORT` in `main.py`.

**"Could not arm Arduino"** — wrong port or sketch not uploaded. Check Device Manager and re-upload if needed.

**Motors don't spin** — check motor power is on. Open Serial Monitor in the Arduino IDE (9600 baud) and confirm you see `READY` on boot.

**`py` not recognised** — try `python` or `python3`.

---

## Files

| File | Purpose |
|---|---|
| `main.py` | Run this for a full test |
| `test_dummy_rover_2.py` | Quick test — use mode 3 for real hardware |
| `rover_combined.ino` | Arduino sketch — upload before anything else |
| `paint_analysis.py` | Post-run image analysis — run separately |
