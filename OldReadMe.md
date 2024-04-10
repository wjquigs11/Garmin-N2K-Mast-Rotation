This repo is a fork of the original mast rotation compensator from RandelO, to enable the code to be used with Garmin wind instruments. The Garmin GNX Wind display does not allow user configuration of its N2K source, so if the GND10 (Nexus<>N2K translator) and the ESP32 (mast rotation corrector) are on the same network, the GNX Wind will always revert to the GND10 and ignore the corrected data.
I solved that problem by using two ESP32 microcontrollers. One sits on an isolated N2K bus with the Nexus/Garmin wind instrument (actually the GND10), and the other sits on the main N2K bus with all the other instruments and displays. The Wind ESP32 processes wind packets from the bus, reads rotation from the Honeywell sensor, and sends corrected wind packets via a serial connection to the other ESP32. The second ESP32 is running a slightly modified version of the Hat Labs NMEA2000 Gateway located here: https://github.com/hatlabs/SH-ESP32-nmea2000-gateway
This ESP32 reads the corrected wind data from its serial port and forwards to the main N2K bus, where it's picked up by the Garmin display.

Start with the Readme here: https://github.com/randel0/NEMA2K-Mast-Rotation-Compensator
I also recommend reading the Hat Labs documentation.
If you just want to build and go, check out the QuickStart.md doc. If you're curious, check the FAQ.md.

Acknowledgements. 
All of the people below provided support for this project and answered many annoying questions.
randelO wrote the original code and figured out how to solve this problem initially for far less than the cost of a commercially-available rotation-compensated masthead wind instrument.
ttlappalainen wrote the N2K libraries.
mairas develops the Sailor Hat ESP32 hardware which is essential for this project, as well as the gateway code.
buhhe got me started on parsing raw N2K data from the 2nd CAN bus.
lkarsten wrote a library to parse Nexus FDX data from the masthead wind transducer.
