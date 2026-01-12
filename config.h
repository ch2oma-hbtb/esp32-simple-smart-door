#ifndef CONFIG_H
#define CONFIG_H

// ============== HARDWARE PIN CONFIGURATION ==============
#define SERVO_PIN 1              // D1 pin on XIAO-ESP32-C3
#define BATTERY_PIN 0            // ADC pin for battery monitoring
#define PAIRING_PIN D7            // D0 pin - short to GND to enter pairing mode

// ============== SERVO CONFIGURATION ==============
// MG90 Servo settings (180 degree servo)
#define DEFAULT_LOCK_ANGLE 0     // Servo position when locked (degrees 0-180)
#define DEFAULT_UNLOCK_ANGLE 120  // Servo position when unlocked (degrees 0-180)
#define SERVO_SPEED 30           // Milliseconds between servo increments (lower = faster)

// ============== BLE SCAN CONFIGURATION ==============
#define BLE_SCAN_TIME 5          // Duration of each BLE scan in seconds
#define BLE_SCAN_INTERVAL 10000  // Interval between scans in milliseconds

// ============== PROXIMITY DETECTION ==============
#define PROXIMITY_THRESHOLD -50  // RSSI threshold (dBm) for unlock trigger
                                 // Higher value (less negative) = closer detection required
                                 // Range typically: -100 (very far) to -30 (very close)
#define RSSI_MIN_THRESHOLD -70   // Minimum RSSI to even register a device
#define DEVICE_DETECTION_TIMEOUT 10000  // Auto-lock if device not detected for this long (ms)

// ============== SECURITY CONFIGURATION ==============
#define MAX_PAIRED_DEVICES 10         // Maximum number of devices that can be paired
#define PAIRING_MODE_TIMEOUT 60000    // Pairing mode timeout (60 seconds)
#define PAIRING_BUTTON_HOLD_TIME 3000 // Hold pairing pin to GND for 3 seconds to enter pairing      

// ============== BATTERY CONFIGURATION ==============
#define BATTERY_CHECK_INTERVAL 30000    // Check battery every 30 seconds (ms)
#define MAX_BATTERY_VOLTAGE 4.2         // 18650 maximum voltage
#define MIN_BATTERY_VOLTAGE 2.8         // 18650 minimum voltage

// ============== DEBUG SETTINGS ==============
#define DEBUG_MODE 1             // 1 = enable debug output, 0 = disable
#define SERIAL_BAUD_RATE 115200

#endif // CONFIG_H
