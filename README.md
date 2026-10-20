# sgco-watch
ESP32 smartwatch with MAX30102 pulse oximeter: BPM, SpO₂, HRV, breathing rate, OLED display and live web page

OVERVIEW
--------
This project uses an ESP32, a MAX30102/MAX30105-compatible optical sensor,
and an SSD1306 128x64 I2C OLED display.

When no finger is detected, the OLED displays the time and date. When a finger
is placed on the sensor, the device displays estimated heart rate (BPM), SpO2,
rhythm status, and a live pulse waveform. Removing the finger returns the
display to the watch face.

The firmware also provides a local Wi-Fi access point and a browser-based
dashboard showing live readings, a pulse waveform, recent heart-rate history,
HRV (RMSSD), and an experimental respiratory-rate estimate.

IMPORTANT: This is an experimental hobby/learning project, not a medical
device. Do not use its readings for diagnosis or treatment decisions.

HARDWARE
--------
- ESP32 development board
- MAX30102/MAX30105 optical pulse sensor breakout
- SSD1306 128x64 I2C OLED display
- USB cable and suitable USB power source
- Jumper wires

WIRING (ESP32 DEVKIT)
---------------------
Connect both the sensor and OLED to the ESP32 I2C bus:

- Sensor GND  -> ESP32 GND
- OLED GND    -> ESP32 GND
- Sensor VIN  -> ESP32 3.3V
- OLED VCC    -> ESP32 3.3V 
- Sensor SDA  -> GPIO 21
- OLED SDA    -> GPIO 21
- Sensor SCL  -> GPIO 22
- OLED SCL    -> GPIO 22

Do not connect a module to 3.3V unless its board supports that supply.

SOFTWARE REQUIREMENTS
---------------------
- Arduino IDE
- ESP32 board support installed in Arduino IDE
- MAX30105 library (SparkFun)
- Adafruit SSD1306 library
- Adafruit GFX Library
- ESP32 libraries included with the selected ESP32 board package

The sketch also uses standard ESP32/Arduino facilities for I2C, Wi-Fi,
web-server functionality, time synchronization, and saved preferences.

WI-FI AND WEB DASHBOARD
-----------------------
At startup, the device attempts to connect to the configured Wi-Fi network to
synchronize the clock. It then starts its own access point for the dashboard.

- Access-point name configured in the sketch: SGCo-Watch
- Dashboard address: http://192.168.4.1
- Access-point password: use the value configured in the sketch

Connect a phone or computer to the device's access point, then open the
dashboard address in a browser. The dashboard may show heart rate, SpO2, HRV,
respiratory-rate estimates, and a pulse waveform.

Anyone who can join the access point may be able to view readings. Change the
default access-point password before sharing or demonstrating the device.

TIME SETTING
------------
The firmware tries several network methods to obtain the current time. If
automatic synchronization fails, use the Serial Monitor at 9600 baud with
line ending set to Newline.

MEASUREMENT NOTES
-----------------
- Keep the finger still and seated consistently over the sensor.
- Allow time for the readings to stabilize.
- Motion, ambient light, sensor fit, skin perfusion, and hardware differences
  can affect the results.
- SpO2 and respiratory rate are estimates and may be inaccurate.
- HRV (RMSSD) requires a sufficiently long period of steady, validated beats.
- The firmware samples the sensor at approximately 50 samples per second after
  averaging, according to the current sketch settings.

TROUBLESHOOTING
---------------
OLED is blank:
- Check power, GND, SDA, and SCL.
- Confirm the OLED I2C address is 0x3C, as configured in the sketch.
- Check that the selected ESP32 board and port are correct.

Sensor not found:
- Check the sensor's power and shared I2C wiring.
- Verify that the breakout is compatible with the library and configured bus.
- Check Serial Monitor for the startup error.

Time does not synchronize:
- Verify the configured network credentials and network access.
- Some campus networks require enterprise authentication and may not work with
  a simple SSID/password connection.
- Use the manual time commands above if needed.

Readings are unstable or show no finger:
- Keep the finger still and correctly positioned.
- Check sensor wiring and power.
- Avoid bright light entering around the fingertip.
- Allow the firmware time to detect the finger and stabilize the signal.

PROJECT STATUS
--------------
This README describes the features and setup reflected in the current sketch.
Actual performance depends on the hardware, libraries, Wi-Fi network, and
firmware configuration.
