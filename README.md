# esp32_xiaomi_m365
Sample Project for decoding serial bus data on a Xiaomi M365 Scooter on Espressif ESP32 / ESP8266

# Wiring
M365 has a Serial One Wire Bus between BLE Module and ESC which consists of 4 wires, the connection as seen on the BLE Module:
- Ground  (Black, "G")
- One Wire Serial Connection (Yellow, "T", 115200bps, 8n1)
- VBatt (Green, "P", always available)
- 5V (Red, "5", only when scooter is turned on)

ESP32/8266 need a Vcc of 3.3V, while at the same time the GPIO Pins are 5V save, so you can wire the 5V to a Vreg for 3.3v which feed the ESP, while the Serial Connection can be wired to RX/TX Pins.
It might be a idea to use e.g. 680R or 1k in series to protect the gpio, as well as add a diode from rx in series with a ~100-200R towards TX

# Working
- Wlan Client/AP with Timeouts
- Telnet on Port 36523 with different Screens
- Telnet on Port 36524 with raw byte dump from M365 Bus
- M365 Serial Receiver and Packet decoder into Array per Address


# Todos
 - finish packet decoder
 - add packet requestor for periodically requesting array-block we are interested in
 - Test on ESP8266
 - add OLED for scooter-mounting
 - dual OLED Screen mode
 - use esp32 dual core features for serial/oled handling on different cores
 - add cheap menu using brake/throttle for navigation
 - add advanced menu using esp32 touch features for navigation

# Telnet Interface
 - Telemetrie Screen shows decoded known values: Batt Voltage, Current, Speed,... 
 - Statistics Screen dumps some internal counters, e.g. Packet Counters, CRC Files, Timing,...
 - ESC/BLE/BMS/"X1" RAW Screens dumps the 512 Byte Array for each device (format "00 00 ...")
 - ESC/BLE/BMS/"X1" Array Screens dumps the 512 Byte Array for each device (in copy & paste format "const bledata[512]={0,0,0...};")

use the letters s,t,e,b,n,x,E,B,N,X to switch between the screens
