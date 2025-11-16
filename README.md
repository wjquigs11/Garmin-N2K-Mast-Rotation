# Garmin N2K Mast Rotation Compensator

This project is a fork of the original mast rotation compensator from RandelO, modified to enable the code to be used with Garmin wind instruments. The solution addresses a specific challenge with Garmin GNX Wind displays, which do not allow user configuration of their N2K source. When the GND10 (Nexus<>N2K translator) and the ESP32 (mast rotation corrector) are on the same network, the GNX Wind will always revert to the GND10 and ignore the corrected data.

## Project Overview

The solution uses an ESP32 microcontroller in a Raspberry Pi form factor with an RS232 CAN bus module. The ESP32 sits on an isolated N2K bus with the Nexus/Garmin wind instrument (GND10), processes wind packets, reads rotation from the Honeywell sensor, and sends corrected wind packets to the main N2K bus, where it's picked up by the Garmin display.

Because the ESP32 is relatively idle, I also use it to process incoming NMEA0183 data from GPS and/or AIS, as well as from a dual-antenna RTK GPS (Unicore UM982) that allows precise heading measurement to compensate for errors in wind angle measurement. It can optionally log tracks to GPX files. It also transmits wind data as NMEA0183 to a controller for my Autohelm tiller pilot so it works in "steer to wind" mode. There's also an environmental sensor, and several web pages to track temperature, humidity, and barometric pressure.

## Do As I Say, Not As I Do

I wouldn't do it this way again! I found the Pi-shaped ESP32 to be quite difficult to work with, considering its triple indirection from Pi GPIOs to Pi header pins to ESP32 GPIOs. Once I got it working, I didn't want to touch it. So why use it? When I started building this system, it was the only practical way to get two isolated CAN interfaces on a single ESP32. All attempts to use the MCP2515 CAN bus module on an ESP32 failed. The simplest solution, which I would implement today, would be to use dual ESP32s connected by UART. One ESP32 reads wind data from Garmin/Nexus and sends it over UART to the other ESP32, which corrects for rotation and transmits the corrected data on the primary boat bus. I coded this solution, but I had no reason to swap out the older working system.

I also experimented with two BNO08X IMUs: one mounted on the mast and one in the ESP32 enclosure in the cabin. This is a decent solution but the IMUs usually have several degrees of error, so I didn't consider it accurate enough. It's cheaper than the Honeywell position sensor, but not worth the savings IMO. It is working fine on my friend's boat.

## Hardware Requirements

- ESP32 microcontroller in Raspberry Pi form factor
- RS232 CAN bus module (https://www.skpang.co.uk/collections/hats/products/pican-m-with-can-bus-micro-c-and-rs422-connector-3a-smps)
- Honeywell position sensor (100° or 180° - 180° recommended if you rotate your mast fully)
- Voltage divider components for the Honeywell sensor
- ADS1015 or 1115 analog to digital converter for the Honeywell sensor

## Configuration Options

The project supports multiple hardware configurations through build flags in platformio.ini:

### Main Build Environments
- **pican-m**: Original version with BNO08X compass, NMEA0183, RTK, display support, and Honeywell sensor
- **rs485can**: Version with gyro mast sensor, BNO08X compass, and display support
- **sh-esp**: Simplified version for Sailor Hat ESP32 with BNO08X compass

### Key Feature Flags
- **PICAN**: Enables MCP2515-based CAN bus interface
- **RS485CAN**: Enables reading heading PGN on wind bus
- **BNO08X**: Configures BNO08X compass at specified I2C address
- **NMEA0183**: Enables NMEA0183 protocol support
- **RTK**: Enables RTK GPS support
- **N2K**: Enables NMEA2000 protocol support
- **DISPLAYON**: Enables OLED display support
- **HONEY**: Enables Honeywell position sensor support
- **WINDLOG**: Enables wind data logging

## Key Features

### Wind Correction
- Reads apparent wind data from NMEA 2000 bus
- Applies mast rotation compensation using Honeywell position sensor
- Calculates true wind angle and speed
- Transmits corrected wind data on NMEA 2000 bus

### Compass Integration
- BNO085 compass support for heading data
- Optional mast-mounted compass for rotation detection

### Data Logging
- Wind data logging capability (when WINDLOG flag enabled)
- Console logging

### Web Interface
- Real-time wind and navigation data display
- Settings configuration
- Calibration tools
- Multiple display pages (wind, compass, settings)

### Connectivity
- NMEA 2000 bus integration
- NMEA 0183 support (when enabled)
- WiFi connectivity for configuration and monitoring
- WebSerial for remote diagnostics

## Core Components

### Sensor Systems
- **Honeywell Position Sensor**: Detects mast rotation angle (when HONEY flag enabled)
- **BNO085 Compass**: Provides heading data
- **Wind Instruments**: Connected via NMEA 2000 or through GND10 translator

### Data Processing
- Wind angle correction based on mast rotation
- True wind calculation from apparent wind and boat speed
- Heading determination from compass modules

### Communication
- NMEA 2000 protocol for marine data exchange
- NMEA 0183 output for legacy devices (when enabled)
- WiFi for web interface and configuration

## Software Architecture

### Core Modules
- **BNO085Compass**: Interface for BNO085 IMU sensor
- **BoatData**: Data structures for vessel information
- **WindSensor**: Wind data processing and correction
- **NMEA Handlers**: Processing of NMEA 2000 and NMEA 0183 messages

### Web Interface
- Asynchronous web server
- Real-time data updates via Server-Sent Events
- Configuration pages for system settings
- Data visualization for wind and navigation data

### Configuration System
- Persistent settings storage using Preferences
- WiFi configuration with fallback to AP mode
- Calibration tools for sensors

## Function Reference

### Wind Processing
- `WindSpeed()`: Processes wind data, applies rotation correction
- `calcTrueWindAngle()`: Calculates true wind angle from apparent wind and boat speed
- `calcTrueWindDirection()`: Calculates true wind direction relative to north
- `readAnalogRotationValue()`: Reads rotation angle from Honeywell sensor (when HONEY flag enabled)

### NMEA Processing
- `HandleNMEA2000MsgMain()`: Processes messages on main N2K bus
- `HandleNMEA2000MsgWind()`: Processes messages on wind N2K bus (when PICAN flag enabled)
- `ParseWindN2K()`: Parses wind data from N2K messages
- `ParseCompassN2K()`: Parses compass data from N2K messages

### Compass Handling
- `getHeading()`: Gets heading from compass module
- `calculateHeading()`: Calculates heading from quaternion data
- `readCompassDelta()`: Calculates difference between mast and boat compass

### Data Logging
- `writeWindPoint()`: Logs wind data to file (when WINDLOG flag enabled)

### Web Interface
- `startWebServer()`: Initializes web server
- `getSensorReadings()`: Prepares sensor data for web interface
- `WebSerialonMessage()`: Handles WebSerial commands

### WiFi Configuration
- `initWiFi()`: Initializes WiFi connection
- `startAP()`: Starts access point mode if WiFi connection fails
- `readWiFi()`: Reads WiFi configuration from file

## Installation and Setup

1. Purchase required hardware components as listed in QuickStart.md
2. Flash the appropriate code onto the ESP32 based on your configuration
3. Connect the Honeywell sensor to the ESP32
4. Configure the system using the web interface

For detailed installation instructions, refer to QuickStart.md.

## Configuration

The system can be configured through:
1. Web interface (accessible via WiFi)
2. WebSerial commands
3. Configuration files on SPIFFS

Key configuration parameters include:
- WiFi settings
- Sensor orientations
- Compass calibration
- Magnetic variation
- Display settings

## Troubleshooting

Common issues and solutions:
- If no wind data appears, check N2K connections and device addresses
- For compass calibration issues, use the calibration page in the web interface
- If WiFi connection fails, the system will start in AP mode for configuration

For more detailed troubleshooting, refer to FAQ.md.

## Credits

This project builds on work from:
- randelO: Original mast rotation compensator code
- ttlappalainen: N2K libraries
- mairas: Sailor Hat ESP32 hardware and gateway code
- buhhe: Parsing raw N2K data
- lkarsten: Library for Nexus FDX data parsing

## License

See LICENSE file for details