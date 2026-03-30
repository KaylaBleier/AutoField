the folder "autofield.3" is a copy of the code I have on laptop with the most updated and necessary code for path planning & path following

To run the code, download the folder and navigate to it in your terminal. Move out the arduino code. 
Be sure the arduino code is properly placed in ARDUINO folder (you should have from downloading the Arduino IDE). 

This test is for using your laptop on the rover to collect precision data. Only once we confirm it working do we move it onto the Pi. 
1) load arduino code to arduino WITH POWER TO MOTORS OFF
2) power system on, attach computer to rover GPS and arduino
3) open device manager & note what port each item is connected to. Take note of Baud rate too but it should be 57600
4) run in your terminal -python main.py (once you're loaded in to the autofield.3 folder)
5) enter requested information 
6) put laptop on rover and let it rip!

If it doens't stop when it reaches destination, stop the system by unplugging arduino 
IF you need to stop the system at any point in set up, press ctrl+c
