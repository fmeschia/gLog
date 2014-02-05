gLog
====
Francesco Meschia, 2013

An open-source acceleration logger. 

Based on the OpenLog platform (Atmel ATmega 328P microcontroller with Arduino bootloader) and on the Anaalog Devices ADXL345 triple-axis accelerometer. 

Features:
---------
- automatic estimation of the aicraft pitch, roll and yaw axes reference frame at startup, only requires to orient the sensing XZ plane to contain the expected direction of motion
- rotation of all measurements into the aircraft reference frame 
- data recording to SD card
- experimental support for automatic launch, zoom and landing recognition for contest timing

