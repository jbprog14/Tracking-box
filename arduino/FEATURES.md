Tracking Box Device

Components: - SHT30 - SIM7600 GNSS (Location) - ESP32 - Accelerometer 6-axis LSM6DSL - Li-Po Battery - 3.7 waveshare E-ink Display - Buzzer

Core Functionality:

- When the Limit Switch is clicked, The device will wake up and sends the data of all sensors to the database, get the details from the database and display them on waveshare 3.7 inch e-ink display, Fit all sensorData and details on our e-ink display respectively

_crucial information: under this wake mode, if the gps readings doesn't match currentLocation and setLocation from the database, ALARM the Buzzer. this means our device has been breached. If they matched, don't sound the Buzzer._

- If there's no network our device currently connected to, Display a Qr Code on our e-ink display that when scanned it will direct me to a page that needed to login to see full description of the package(Item Description and all available details from the firebase), This functionality required a basic Username and Password to go through the qr-code link to view the details, You can generate both qr-code and simple account verification, assumingly this means you need to edit the page.tsx

- Our Accelerometer will constantly listen for interruption despite Esp32's sleep or wake mode, And if it did received an interruption or sudden tilting, wake the esp32 and repeat the data sending and detail displaying process, Additionally, on our website create me a simple alert notification indicating this event.

- IMPORTANT : On Every 15 minutes interval ESP32 will wake, unless it got interrupted by Accelerometer even if it's not 15 minutes passed, wake ESP32. \*

- We can utilize the existing tracking_box table on our database linked to this device
