# XIAO-ESP32-C3 Smart Door Lock Firmware

A BLE-based smart door lock firmware for Seeed Studio XIAO-ESP32-C3 with proximity detection and servo control.

## Features

- **BLE Proximity Detection**: Automatically unlocks when authorized BLE devices come within range
- **Servo Motor Control**: Uses ESP32 Servo Library to control MG90 180° servo motor
- **Configurable Angles**: Easily adjust lock/unlock servo positions
- **Auto-Lock**: Automatically re-locks after device leaves proximity or after timeout
- **Battery Monitoring**: Tracks 18650 battery voltage and percentage
- **Low Power Passive BLE Scanning**: Efficient power consumption for battery operation

## Hardware Requirements

- **Board**: Seeed Studio XIAO-ESP32-C3
- **Servo Motor**: MG90 (180°)
- **Power**: 18650 Li-ion Battery (3.7V nominal)
- **Connections**:
  - Servo Signal → Pin D1 (GPIO1)
  - Servo Power → 5V (via voltage regulator if needed)
  - Servo GND → GND
  - Battery + → BAT/5V (through protection circuit recommended)
  - Battery - → GND

## Pin Mapping

| Component | Pin | Notes |
|-----------|-----|-------|
| Servo Signal | D1 (GPIO1) | PWM output |
| Battery ADC | D0 (GPIO0) | Analog input for voltage monitoring |
| GND | GND | Common ground |

## Installation

1. **Install Arduino IDE** and **ESP32 board support**:
   - Board: ESP32 Dev Module (or XIAO-ESP32-C3 if available)
   - Driver: CP2102 USB-to-Serial

2. **Install Required Libraries**:
   ```
   - ESP32Servo (Jarrett Bump)
   - ESP32 BLE Arduino (built-in with ESP32 core)
   ```

3. **Upload the Firmware**:
   - Select board: "ESP32C3 Dev Module" or equivalent
   - Select port: /dev/tty.usbserial-*
   - Upload `SmartDoorLock.ino`

## Configuration

Edit `config.h` to customize behavior:

### Servo Angles
```cpp
#define DEFAULT_LOCK_ANGLE 0      // Locked position
#define DEFAULT_UNLOCK_ANGLE 90   // Unlocked position
```

### BLE Proximity Threshold
```cpp
#define PROXIMITY_THRESHOLD -50   // RSSI in dBm
// -30 dBm: Very close (1-2 meters)
// -50 dBm: Close (3-5 meters)
// -70 dBm: Far (5-10 meters)
```

### Auto-Lock Timeout
```cpp
#define DEVICE_DETECTION_TIMEOUT 10000  // milliseconds
```

## Behavior

1. **Initialization**: Door starts in **LOCKED** position
2. **BLE Scanning**: Device scans for nearby BLE devices every 10 seconds
3. **Detection**: When a device with RSSI > -50 dBm is detected:
   - Door **UNLOCKS** (servo moves to unlock angle)
   - Remains unlocked while device is in range
4. **Timeout**: If no device detected for 10 seconds:
   - Door **LOCKS** (servo returns to lock angle)
5. **Battery**: Voltage monitored on GPIO0 (optional, requires ADC wiring)

## Serial Output Example

```
SmartDoorLock Firmware Starting...
Initializing Servo...
Initializing BLE...
Setup Complete!
Lock Angle: 0° | Unlock Angle: 90°
Proximity Threshold: -50 dBm

[BLE] Starting scan...
[BLE] Scan completed. Devices found: 2
Device detected: 11:22:33:44:55:66 | RSSI: -45
[ACTION] Unlocking door...
[SERVO] Door is now UNLOCKED

[BATTERY] Voltage: 4.15V | Percentage: 87.5%

[ACTION] Locking door...
[SERVO] Door is now LOCKED
```

## Troubleshooting

### Servo Not Moving
- Check D1 pin connection
- Verify servo power supply (5V recommended)
- Test with servo at known angles: 0°, 45°, 90°, 180°

### BLE Not Detecting Devices
- Ensure target device has BLE enabled and advertising
- Check `PROXIMITY_THRESHOLD` value (less negative = closer detection)
- Monitor serial output for detected devices

### Battery Not Reading
- Verify ADC pin connection (GPIO0)
- Check voltage divider resistor values if implemented
- Ensure battery voltage is between 2.8V - 4.2V

### Door Stuck at One Position
- Manually check servo for mechanical binding
- Test servo movement independently
- Try adjusting lock/unlock angles in config.h

## Advanced Configuration

### Modify Servo Speed
Lower value = faster movement:
```cpp
#define SERVO_SPEED 30  // milliseconds between increments
```

### Change Scan Interval
For faster response (higher power usage):
```cpp
#define BLE_SCAN_INTERVAL 5000  // scan every 5 seconds
```

### Fine-Tune RSSI Range
Check actual RSSI values in serial monitor and adjust:
```cpp
setProximityThreshold(-45);  // Call in setup() or loop()
```

## Power Consumption

- **Active Scanning**: ~80-100 mA
- **Idle State**: ~20-30 mA
- **Servo Moving**: ~500+ mA (peak)

Typical 18650 (3000 mAh) provides ~20-40 hours runtime depending on usage patterns.

## Safety Considerations

⚠️ **Important**: 
- Add a **mechanical override lock** independent of electronics
- Implement **emergency unlock** button
- Use a **diode** to prevent reverse polarity from battery
- Consider **fuse/PTC protection** on power line
- Test all scenarios before deploying

## License

MIT License - Feel free to modify and use for your projects

## Support

For issues or suggestions, check the serial monitor output and verify:
1. ESP32 board is recognized by Arduino IDE
2. USB cable is data-capable (not charge-only)
3. All libraries are properly installed
4. Battery voltage is adequate (> 3.0V)
