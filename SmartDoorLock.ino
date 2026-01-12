#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Preferences.h>

#include "config.h" 

// ============== PIN CONFIGURATION ==============
#define SERVO_PIN 1  // D1 pin on XIAO-ESP32-C3
#define BATTERY_PIN 0  // ADC pin for battery monitoring (optional)

// ============== SERVO CONFIGURATION ==============
Servo doorServo;
int lockAngle = DEFAULT_LOCK_ANGLE;        
int unlockAngle = DEFAULT_UNLOCK_ANGLE;    
int currentAngle = DEFAULT_LOCK_ANGLE;

// ============== BLE CONFIGURATION ==============
BLEScan* pBLEScan;
int scanTime = BLE_SCAN_TIME;  
int scanInterval = BLE_SCAN_INTERVAL; 
unsigned long lastScanTime = 0;

// ============== PROXIMITY DETECTION ==============
int proximityThreshold = PROXIMITY_THRESHOLD;  
int rssiThreshold = RSSI_MIN_THRESHOLD;  

// ============== LOCK/UNLOCK LOGIC ==============
bool doorLocked = true;
bool deviceDetected = false;
unsigned long deviceDetectionTime = 0;
unsigned long lockTimeout = DEVICE_DETECTION_TIMEOUT;  

// ============== BATTERY MONITORING ==============
float batteryVoltage = 0.0;
float batteryPercentage = 0.0;
// Constants defined in config.h

// ============== PAIRING MODE ==============
Preferences preferences;
bool pairingMode = false;
unsigned long pairingModeStartTime = 0;
String pairedDevices[MAX_PAIRED_DEVICES];
int pairedDeviceCount = 0;
unsigned long pairingButtonPressTime = 0;
bool pairingButtonPressed = false;

// Forward declarations
bool isDevicePaired(String macAddress);
void addPairedDevice(String macAddress);

// BLE scan callback class
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Only process devices during pairing mode
    if (!pairingMode) {
      return;
    }
    
    String devName = advertisedDevice.getName().c_str();
    String devMac = advertisedDevice.getAddress().toString().c_str();
    int devRssi = advertisedDevice.getRSSI();

    // Debug: Show devices found during pairing
    Serial.print("[FOUND] Name: '");
    Serial.print(devName);
    Serial.print("' | MAC: ");
    Serial.print(devMac);
    Serial.print(" | RSSI: ");
    Serial.println(devRssi);

    // Add new device if not already paired
    if (!isDevicePaired(devMac)) {
      addPairedDevice(devMac);
      Serial.print("[PAIR] ✓ Device paired: ");
      Serial.print(devName);
      Serial.print(" | MAC: ");
      Serial.println(devMac);
    } else {
      Serial.println("[PAIR] Device already paired.");
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\nSmartDoorLock Firmware Starting...");
  
  // Initialize pairing button pin
  pinMode(PAIRING_PIN, INPUT_PULLUP);
  
  // Initialize preferences (flash storage)
  preferences.begin("doorlock", false);
  loadPairedDevices();
  
  Serial.print("[PAIR] Loaded ");
  Serial.print(pairedDeviceCount);
  Serial.println(" paired device(s)");
  
  // Initialize servo
  Serial.println("Initializing Servo...");
  doorServo.attach(SERVO_PIN);
  setLock(true);  // Start in locked position
  
  // Initialize BLE
  Serial.println("Initializing BLE...");
  BLEDevice::init("SmartDoorLock");
  
  // Create BLE scan object
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true); // true = want duplicates? No, usually false is fine. wait, the param is 'wantDuplicates'. Keep false? Actually the second param to setAdvertisedDeviceCallbacks is (wantDuplicates). 
  // Let's check documentation. Standard ESP32 BLE: setAdvertisedDeviceCallbacks(callbacks, wantDuplicates)
  // But wait, to get NAMES we need Active Scanning.
  
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true); // allow duplicates to track RSSI updates
  pBLEScan->setActiveScan(true);  // Active scan (requests scan response for NAMES)
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  
  Serial.println("Setup Complete!");
  Serial.println("\n========================================");
  Serial.println("Smart Door Lock - Pairing Mode Only");
  Serial.println("========================================");
  Serial.println("To pair devices:");
  Serial.print("1. Short D");
  Serial.print(PAIRING_PIN);
  Serial.print(" to GND for ");
  Serial.print(PAIRING_BUTTON_HOLD_TIME / 1000);
  Serial.println(" seconds");
  Serial.println("2. Bring your BLE device close");
  Serial.println("3. Device will be paired automatically");
  Serial.print("\nPaired devices: ");
  Serial.println(pairedDeviceCount);
  Serial.println("========================================\n");
}

void loop() {
  // Check pairing button
  checkPairingButton();
  
  // Handle pairing mode
  if (pairingMode) {
    // Check if pairing mode has timed out
    if (millis() - pairingModeStartTime > PAIRING_MODE_TIMEOUT) {
      exitPairingMode();
    } else {
      // Perform BLE scan during pairing mode
      if (millis() - lastScanTime >= 2000) { // Scan every 2 seconds during pairing
        lastScanTime = millis();
        performBLEScan();
      }
    }
  }
  
  // Update battery status
  updateBatteryStatus();
  
  delay(100);
}

void performBLEScan() {
  Serial.println("[BLE] Scanning for devices...");
  BLEScanResults* foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("[BLE] Scan completed. Devices found: ");
  Serial.println(foundDevices->getCount());
  pBLEScan->clearResults();
}

void setLock(bool lock) {
  int targetAngle = lock ? lockAngle : unlockAngle;
  
  // Smooth servo movement
  if (currentAngle < targetAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle += 5) {
      doorServo.write(angle);
      delay(30);
    }
  } else {
    for (int angle = currentAngle; angle >= targetAngle; angle -= 5) {
      doorServo.write(angle);
      delay(30);
    }
  }
  
  doorServo.write(targetAngle);
  currentAngle = targetAngle;
  doorLocked = lock;
  
  Serial.print("[SERVO] Door is now ");
  Serial.println(lock ? "LOCKED" : "UNLOCKED");
}

void updateBatteryStatus() {
  // Read battery voltage from ADC (if ADC pin is available)
  // Note: XIAO-ESP32-C3 ADC input should be routed through voltage divider
  // For example: 18650 -> Resistor Divider (100k + 100k) -> GPIO pin
  
  int rawValue = analogRead(BATTERY_PIN);
  batteryVoltage = (rawValue / 4095.0) * 3.3 * 2;  // Assuming 1:1 voltage divider
  
  // Calculate battery percentage
  batteryPercentage = ((batteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100;
  batteryPercentage = constrain(batteryPercentage, 0, 100);
  
  // Optional: Print battery status periodically
  static unsigned long lastBatteryPrint = 0;
  if (millis() - lastBatteryPrint >= 30000) {  // Print every 30 seconds
    lastBatteryPrint = millis();
    Serial.print("[BATTERY] Voltage: ");
    Serial.print(batteryVoltage, 2);
    Serial.print("V | Percentage: ");
    Serial.print(batteryPercentage, 1);
    Serial.println("%");
  }
}

// ============== CONFIGURATION FUNCTIONS ==============
// Adjust these functions to fine-tune behavior

void setProximityThreshold(int rssi) {
  proximityThreshold = rssi;
  Serial.print("[CONFIG] Proximity threshold set to: ");
  Serial.println(rssi);
}

void setLockAngles(int lockPos, int unlockPos) {
  lockAngle = lockPos;
  unlockAngle = unlockPos;
  Serial.print("[CONFIG] Lock angle: ");
  Serial.print(lockAngle);
  Serial.print("° | Unlock angle: ");
  Serial.println(unlockAngle);
}

void setLockTimeout(unsigned long timeout) {
  lockTimeout = timeout;
  Serial.print("[CONFIG] Lock timeout set to: ");
  Serial.print(timeout);
  Serial.println(" ms");
}

// ============== PAIRING MODE FUNCTIONS ==============

void checkPairingButton() {
  bool buttonPressed = (digitalRead(PAIRING_PIN) == LOW);
  
  if (buttonPressed && !pairingButtonPressed) {
    // Button just pressed
    pairingButtonPressed = true;
    pairingButtonPressTime = millis();
  } else if (!buttonPressed && pairingButtonPressed) {
    // Button released
    pairingButtonPressed = false;
  } else if (buttonPressed && pairingButtonPressed) {
    // Button held
    unsigned long holdTime = millis() - pairingButtonPressTime;
    if (holdTime >= PAIRING_BUTTON_HOLD_TIME && !pairingMode) {
      enterPairingMode();
    }
  }
}

void enterPairingMode() {
  pairingMode = true;
  pairingModeStartTime = millis();
  Serial.println("\n========================================");
  Serial.println("[PAIR] PAIRING MODE ACTIVATED!");
  Serial.println("[PAIR] Bring your device close to pair...");
  Serial.print("[PAIR] Timeout in ");
  Serial.print(PAIRING_MODE_TIMEOUT / 1000);
  Serial.println(" seconds");
  Serial.println("========================================\n");
}

void exitPairingMode() {
  pairingMode = false;
  Serial.println("\n[PAIR] Pairing mode ended.");
  Serial.print("[PAIR] Total paired devices: ");
  Serial.println(pairedDeviceCount);
}

bool isDevicePaired(String macAddress) {
  for (int i = 0; i < pairedDeviceCount; i++) {
    if (pairedDevices[i] == macAddress) {
      return true;
    }
  }
  return false;
}

void addPairedDevice(String macAddress) {
  if (pairedDeviceCount >= MAX_PAIRED_DEVICES) {
    Serial.println("[PAIR] ERROR: Maximum paired devices reached!");
    return;
  }
  
  if (isDevicePaired(macAddress)) {
    return; // Already paired
  }
  
  pairedDevices[pairedDeviceCount] = macAddress;
  pairedDeviceCount++;
  savePairedDevices();
}

void loadPairedDevices() {
  pairedDeviceCount = preferences.getInt("deviceCount", 0);
  for (int i = 0; i < pairedDeviceCount; i++) {
    String key = "device" + String(i);
    pairedDevices[i] = preferences.getString(key.c_str(), "");
    if (DEBUG_MODE) {
      Serial.print("[PAIR] Loaded: ");
      Serial.println(pairedDevices[i]);
    }
  }
}

void savePairedDevices() {
  preferences.putInt("deviceCount", pairedDeviceCount);
  for (int i = 0; i < pairedDeviceCount; i++) {
    String key = "device" + String(i);
    preferences.putString(key.c_str(), pairedDevices[i]);
  }
  Serial.println("[PAIR] Paired devices saved to flash.");
}

void clearAllPairedDevices() {
  pairedDeviceCount = 0;
  preferences.clear();
  Serial.println("[PAIR] All paired devices cleared!");
}
