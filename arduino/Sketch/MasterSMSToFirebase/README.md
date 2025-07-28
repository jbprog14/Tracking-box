# Master SMS-to-Firebase Device

## Overview

This system consists of two main components that enable remote tracking devices to send sensor data to Firebase when WiFi connectivity is unavailable:

1. **Master Device** (`MasterSMSToFirebase.ino`) - Receives SMS messages and forwards data to Firebase
2. **Remote Sender** (`RemoteSMSSender.ino`) - Sends formatted SMS messages with sensor data

## Architecture

```
Remote Tracking Device(s)
         |
         | SMS Message
         v
    Master Device
         |
         | WiFi/HTTP
         v
    Firebase Database
```

## SMS Message Format

The system uses a comma-separated format for SMS messages:

```
DEVICE_ID,timestamp,temp,humidity,lat,lng,alt,tilt,fall,limitSwitch,accelX,accelY,accelZ,batteryVoltage,wakeUpReason
```

### Example SMS Message:

```
box_002,1703123456789,25.5,60.0,14.562000,121.112100,15.0,0,0,1,0.020,-0.010,0.980,3.85,TIMER DUE (15mns.)
```

### Field Descriptions:

- `DEVICE_ID`: Unique identifier for the tracking device (e.g., "box_001")
- `timestamp`: Unix timestamp in milliseconds
- `temp`: Temperature in Celsius
- `humidity`: Humidity percentage
- `lat`: Latitude (decimal degrees)
- `lng`: Longitude (decimal degrees)
- `alt`: Altitude in meters
- `tilt`: Tilt detected (1=true, 0=false)
- `fall`: Fall/shock detected (1=true, 0=false)
- `limitSwitch`: Limit switch pressed (1=true, 0=false)
- `accelX`: X-axis acceleration in g's
- `accelY`: Y-axis acceleration in g's
- `accelZ`: Z-axis acceleration in g's
- `batteryVoltage`: Battery voltage
- `wakeUpReason`: Reason for device wake-up (string)

## Master Device Setup

### Hardware Requirements:

- ESP32 development board
- SIM7600 module
- SIM card with SMS capability
- WiFi network access

### Pin Connections:

```
ESP32 GPIO 16 -> SIM7600 TX
ESP32 GPIO 17 -> SIM7600 RX
```

### Configuration:

1. Update WiFi credentials in the sketch:

   ```cpp
   const char* WIFI_SSID = "your_wifi_ssid";
   const char* WIFI_PASSWORD = "your_wifi_password";
   ```

2. Verify Firebase configuration matches your project:
   ```cpp
   const char* FIREBASE_HOST = "your-project.firebasedatabase.app";
   const char* FIREBASE_AUTH = "your_api_key";
   ```

### Installation:

1. Install required libraries:

   - ArduinoJson
   - WiFi (ESP32)
   - HTTPClient (ESP32)

2. Upload the `MasterSMSToFirebase.ino` sketch to your ESP32

3. Insert SIM card and power on the device

## Remote Device Integration

### Method 1: Standalone Remote Device

Use the `RemoteSMSSender.ino` sketch as a standalone device:

1. Configure the device ID and master phone number:

   ```cpp
   const String DEVICE_ID = "box_002";
   const String MASTER_PHONE_NUMBER = "+1234567890";
   ```

2. Update the `collectSampleSensorData()` function to read from actual sensors

## Enhanced SMS Processing Workflow

The Master device uses a robust two-step SMS processing method:

### Step 1: List Unread Messages

```
AT+CMGL="REC UNREAD"
```

Returns indices of all unread messages without fully reading content.

### Step 2: Read Specific Messages

```
AT+CMGR=<index>
```

Reads the complete message content for each index found in Step 1.

### Step 3: Process and Delete

- Parse sensor data from message content
- Upload to Firebase database
- Delete message only after successful processing: `AT+CMGD=<index>`

### Benefits of AT+CMGR Approach:

- **Reliable Content Extraction**: Full message content guaranteed
- **Error Recovery**: Messages only deleted after successful Firebase upload
- **Better Parsing**: Structured response format easier to parse
- **Multiple Messages**: Handles multiple unread messages sequentially
- **Debugging**: Complete message content visible for troubleshooting

### Method 2: Integration with Existing Firmware

Add SMS sending capability to existing tracking device firmware:

```cpp
// Add this function to your existing firmware
void sendSensorDataViaSMS() {
  if (WiFi.status() != WL_CONNECTED) {
    // Format sensor data as SMS
    String smsMessage = formatSensorDataForSMS(currentData);

    // Send to master device
    if (sendSMS(MASTER_PHONE_NUMBER, smsMessage)) {
      Serial.println("✓ Sensor data sent via SMS");
    }
  }
}
```

## Firebase Data Structure

The master device creates the following JSON structure in Firebase:

```json
{
  "tracking_box": {
    "box_002": {
      "sensorData": {
        "timestamp": 1703123456789,
        "temp": 25.5,
        "humidity": 60.0,
        "batteryVoltage": 3.85,
        "gpsFixValid": true,
        "limitSwitchPressed": true,
        "tiltDetected": false,
        "fallDetected": false,
        "wakeUpReason": "TIMER DUE (15mns.)",
        "receivedViaSMS": true,
        "masterDevice": "SMS_Gateway",
        "accelerometer": {
          "x": 0.02,
          "y": -0.01,
          "z": 0.98
        },
        "location": {
          "lat": 14.562,
          "lng": 121.1121,
          "alt": 15.0
        },
        "currentLocation": "14.5620, 121.1121"
      }
    }
  }
}
```

## Operation

### Master Device:

1. Continuously monitors for incoming SMS messages using AT+CMGL="REC UNREAD"
2. Reads each message individually using AT+CMGR=<index> for proper content extraction
3. Parses SMS content according to the defined format
4. Converts data to Firebase JSON format
5. Sends HTTP PUT request to Firebase
6. Deletes successfully processed SMS messages using AT+CMGD=<index>
7. Maintains WiFi connection with auto-reconnect

### Remote Device:

1. Collects sensor data
2. Formats data as comma-separated SMS message
3. Sends SMS to master device
4. Handles response confirmation

## Monitoring and Troubleshooting

### Master Device Serial Output:

```
===== MASTER SMS-TO-FIREBASE DEVICE =====
✅ WiFi Connected - Ready to receive SMS
=== UNREAD SMS DETECTED ===
List Response: +CMGL: 1,"REC UNREAD","+639184652918","","23/11/28,15:30:22+32"

Found unread message at index: 1
Reading SMS at index 1 using AT+CMGR...
CMGR Response: +CMGR: "REC UNREAD","+639184652918","","23/11/28,15:30:22+32"
box_001,1703123456789,25.5,60.0,14.562000,121.112100,15.0,0,0,1,0.020,-0.010,0.980,3.85,TIMER DUE (15mns.)

OK
Extracted content: 'box_001,1703123456789,25.5,60.0,14.562000,121.112100,15.0,0,0,1,0.020,-0.010,0.980,3.85,TIMER DUE (15mns.)'
=== NEW SMS RECEIVED ===
SMS Content: box_001,1703123456789,25.5,60.0,14.562000,121.112100,15.0,0,0,1,0.020,-0.010,0.980,3.85,TIMER DUE (15mns.)
✓ SMS data parsed successfully
Device ID: box_001
Parsed: ID=box_001, Temp=25.5°C, Hum=60.0%, GPS=Valid, Batt=3.85V
✓ Firebase PUT successful for box_001, response code: 200
Deleting SMS at index: 1
✓ SMS deleted successfully
=========================
📡 Master device active - WiFi: Connected
```

### Remote Device Serial Output:

```
===== REMOTE SMS SENDER =====
Sample Data - Temp: 25.5°C, Hum: 60.0%, GPS: 14.562000,121.112100, Batt: 3.85V
Formatted SMS: box_002,1703123456789,25.5,60.0,14.562000,121.112100,15.0,0,0,1,0.020,-0.010,0.980,3.85,TIMER DUE (15mns.)
✓ SMS sent successfully
```

## Error Handling

The system includes robust error handling:

- **WiFi Connection Issues**: Master device attempts reconnection
- **SMS Parsing Errors**: Invalid messages are logged and discarded
- **Firebase Communication Errors**: HTTP response codes are logged
- **SIM Module Issues**: AT command responses are monitored

## Cost Considerations

- **SMS Costs**: Each sensor reading requires one SMS message
- **Data Usage**: Master device uses minimal WiFi data for Firebase uploads
- **Power Consumption**: Remote devices can use low-power modes between transmissions

## Limitations

- **SMS Character Limit**: Messages must fit within 160 characters
- **Delivery Latency**: SMS delivery may have delays
- **Network Dependencies**: Master device requires stable WiFi for Firebase access
- **Single Master**: Current implementation supports one master device per phone number

## Future Enhancements

- **Multi-Master Support**: Load balancing across multiple master devices
- **Message Compression**: Reduce SMS size through data compression
- **Batch Processing**: Combine multiple readings in single SMS
- **Error Recovery**: Automatic retry mechanisms for failed transmissions

## Testing the Enhanced SMS System

### Testing AT+CMGR Workflow

1. **Send a test SMS** to the Master device SIM card:

   ```
   box_001,1703123456789,25.5,60.0,14.562000,121.112100,15.0,0,0,1,0.020,-0.010,0.980,3.85,TIMER DUE (15mns.)
   ```

2. **Monitor Serial Output** for the complete workflow:

   - AT+CMGL lists unread messages
   - AT+CMGR reads specific message content
   - Content parsing and Firebase upload
   - AT+CMGD deletes processed message

3. **Verify Firebase Update** in your database:
   - Check `/tracking_box/box_001/sensorData/`
   - Confirm `receivedViaSMS: true` flag
   - Verify `masterDevice: "SMS_Gateway"` identifier

### Phone Number Configuration

The Master device is configured to work with the phone number: **+639184652918**

Make sure to:

- Insert a SIM card with this number in the Master device
- Update remote devices to send SMS to this number
- Verify network coverage for both SMS and WiFi connectivity

### Troubleshooting AT+CMGR Issues

If messages aren't being processed:

1. **Check SIM Card**: Ensure SIM is inserted and has SMS capability
2. **Monitor AT Commands**: Watch for "OK" responses after each command
3. **Verify Message Format**: Ensure SMS content matches expected CSV format
4. **WiFi Connection**: Firebase uploads require active WiFi connection
5. **Storage Space**: SIM card should have space for incoming SMS
