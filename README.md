# Precision-orchard-management

This repository contains assorted files used to support precision orchard management research projects:

- precisionBag.m is example Matlab code for extracting and processing GNSS data from rosbag files
- gnss_sd_logger.ino is a program for recording RTK GPS data using a portable, microcontroller-based system; the adopted architecture is described in further detail below

We developed a small, portable system for recording RTK GPS data in agricultural environments. The system's architecture is shown in the following block diagram.

<img width="3563" height="935" alt="GnnsArchitecture" src="https://github.com/user-attachments/assets/85d049e4-f161-4622-be07-0dc0ce321e17" />

The enclosure and hardware design are also shown here

<img width="2313" height="1734" alt="Picture2" src="https://github.com/user-attachments/assets/05ce00ba-c273-4d6e-a649-94351510f6e3" />

A few 'low-level' lessons learned during development (July 2026):
- The Sparkfun BlueSmirf v2 arrived from Amazon without firmware installed. Hold down the module's pair button while powering on to enter bootloading mode. Use a Sparkfun serial basic device to flash the firmware, following the steps online from Sparkfun (Claude assisted with this step). After the firmware is uploaded, put the Bluesmirf in low energy (BLE) mode using AT commands (only required if using an ios device). 
- The BlueSmirf is not discoverable through the iphone’s settings menu. Need to connect to the module through the LightBlue iOS app; verify that you can write and subscribe to the GNSS receiver on 2 peripherals (you should see the NMEA messages coming in if there is a valid connection). Afterwards, disconnect from LightBlue; the BlueSmirf should now be discoverable through SW Maps.
- BLE will get overwhelmed and disconnect after ~30-60 seconds if passing all available NMEA data to SW Maps. To improve the connection, go to u-center and remove some of the unnecessary data from getting passed through the receiver’s UART 2 port (Claude assisted with this step). The most important thing is to remove all of the satellite data. Also, make sure that the sample rate is set to 1 Hz and the Baud rates match everywhere in the system (e.g., receiver, BlueSmirf, etc.) 
- Make sure that NMEA data is sent back to the ORGN or Skylark NTRIP caster (toggle on the switch in SW Maps).
- Updating and saving the receiver configuration in u-center can be tricky. Make sure that it gets saved to permanent memory. 
- Establish a standard 3D fix and let it stabilize for a minute before attempting an RTK fix.
- Using I2C and an Arduino Mega with the Sparkfun library to communicate with the receiver was pretty seamless. I used the 3.3 V pin on the Arduino for power to the receiver. The board processes the RTCM corrections received over BLE (UART2) automatically. An RTX fix was maintained for about 20 minutes. Note, the Arduino Mega was replaced with a Sparkfun Thing Plus ESP32 Wroom, which has onboard Bluetooth, in the final prototype.
- The microSD card is not accessible if the Sparkfun Thing Plus is mounted to a breadboard. My workaround was to use a microSD card cable extender, but make sure the extender doesn't have a connector that interferes with the USB C cable! 


