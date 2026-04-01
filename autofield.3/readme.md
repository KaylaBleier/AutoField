# Autofield Rover — Test Day

This is a laptop-based run — once we confirm it working here we move to the Pi. This means someone's laptop will need to be on the rover and running the code. It's safe, I did it, dw.

> **Pi note:** No code is on the Pi yet. If Abby tweaked `path_following.py` she should push it to GitHub and everyone pulls from there.  I also caution against testing everything at once — first run without the arm, then test the arm separately. The risk of something frying or snapping is too high to do it all in one go. This code hasn't been run on a full system yet and I really really don't want the rover to break in anyway. 

---

## Setup

Download `autofield.3` and open a terminal inside it.

### Arduino folder

The library version matters here — do NOT update it if the IDE asks you to. It only works on an older version and figuring out which one took a lot of debugging.

To avoid any of that: make a copy of your existing Arduino folder (usually in `Documents/`) and save it to your desktop as a backup. Then replace it with the one from the Drive — it's already set up correctly with the right library versions and all the screen code, so you shouldn't need to change anything.

**[Download the Arduino folder here](https://drive.google.com/file/d/1cgGodQ53Zb1sHAHSOaOvpdaxyKCjckyK/view?usp=drive_link)**
*(it's a ZIP — extract and drop it in `Documents/`)*

All the Arduino code is also in the `screen_code` folder if you need it.

---

## 1. Load the Arduino sketch

> ⚠️ Motors off for this step.

Open `rover_control_4.ino` in the Arduino IDE, select **Tools → Board → Arduino Uno**, pick the right port, and upload.

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

All runs log to a timestamped CSV in `track_logs/`. The data is saved — just upload that folder to the Drive after all testing is done.

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
| `rover_control_4.ino` | Arduino sketch — upload before anything else |
| `paint_analysis.py` | Post-run image analysis — run separately |
