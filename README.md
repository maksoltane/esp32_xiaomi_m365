# esp32_xiaomi_m365
Sample Project for decoding serial bus data on a Xiaomi M365 Scooter on Espressif ESP32 / ESP8266

# Working
- WiFi Auto Connect to known SSIDs or AP-Mode & Telnet with Timeouts to auto-turnoff Telnet/WiFi
- Firmwareupdates (of ESP Device, not the scooter) over WiFi
- Telnet on Port 36523 with different Screens
- Telnet on Port 36524 with raw byte dump from M365 Bus ("read only")
- M365 Serial Receiver and Packet decoder into Array per Address
- Decoder of BMS Data Array to Variable Values/Telemetrie Screen

# Todos
 - finish packet decoder (the ESC Struct is not tested/verified)
 - beautify telemetrie screen (e.g. show "lights on/off" instead of 0x00/0x64)
 - add newdata-bitarray for each address to indicate updated data
 - add packet requestor for periodically requesting array-block we are interested in
 - Test if 3.3Vfrom TX Pin are enough or a simple Level Shifter is needed
 - Test on ESP8266
 - add OLED display code, single/dual OLED Screen mode, screenmodes, auto-refresh on new data via newdataarray
 - use esp32 dual core features for serial/oled handling on different cores
 - add cheap menu using brake/throttle for navigation
 - add advanced menu using esp32 touch features for navigation
 - custom PCB with 2 OLEDs, ESP32S, VReg and touch areas

# Todos 2 - Ideas & Visions:
 - add Scooter-Flashing Protection (so no one can flash broken firmware to your scooter while waiting at a red traffic light
 - advanced thief/lock protection
 - advanced trip computer (which keeps trip-totals/averages between 2 charge cycles or 2 times with the same available SSID (leaving/coming home)
 - MQTT Logging of Trip-Summary Data
 
# Telnet Interface
 - Telemetrie Screen shows decoded known values: Batt Voltage, Current, Speed,... 
 - Statistics Screen dumps some internal counters, e.g. Packet Counters, CRC Files, Timing,...
 - ESC/BLE/BMS/"X1" RAW Screens dumps the 512 Byte Array for each device (format "00 00 ...")
 - ESC/BLE/BMS/"X1" Array Screens dumps the 512 Byte Array for each device (in copy & paste format "const bledata[512]={0,0,0...};")

use the letters
 - s,t,e,b,n,x,E,B,N,X to switch between the screens
 - r to reset statistics & m365 data arrays


# Wiring
M365 has a Serial One Wire Bus between BLE Module and ESC which consists of 4 wires, the connection as seen on the BLE Module:
- Ground  (Black, "G")
- One Wire Serial Connection (Yellow, "T", 115200bps, 8n1)
- VBatt (Green, "P", always available)
- 5V (Red, "5", only when scooter is turned on)

ESP32/8266 needs a Vcc of 3.3V, while at the same time the GPIO Pins are 5V save, so you can wire the 5V to a Vreg for 3.3v which feed the ESP, while the Serial Connection can be wired to RX/TX Pins.
It might be a idea to use e.g. 680R or 1k in series to protect the gpio, as well as add a diode from rx in series with a ~100-200R towards TX

