#include <Arduino.h>

#include <ArduinoBLE.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <algorithm>
#include <vector>

// --- Configuration ---
#define AP_SSID "SmartLock-Setup"
#define AP_PASSWORD "12345678" // Minimum 8 characters for WPA2
#define DEFAULT_RSSI_THRESHOLD -70
#define BLE_SCAN_INTERVAL 100  // Short pause between scans (100ms)
#define BLE_SCAN_DURATION 3000 // Scan for 3 seconds

// Hardware Pin Definitions for Seeed Studio XIAO ESP32-C3
// Note: D0=GPIO2, D1=GPIO3, D10=GPIO10, etc.
#define PIN_SERVO D1 // Connect Servo Signal Here
#define PIN_STATUS_LED                                                         \
  D2 // Connect LED Here (XIAO C3 has no user LED on typical pins, use external)
#define PIN_OPEN_BUTTON D8 // Boot Button on XIAO ESP32-C3
#define PIN_WIFI_RESET                                                         \
  D6 // WiFi Reset Button (short to GND to reset WiFi settings)

// Lock Settings
#define LOCK_OPEN_DURATION_MS 15000 // How long the lock stays open in ms
#define SERVO_LOCKED_ANGLE_DEFAULT 0     // Default servo angle when locked
#define SERVO_UNLOCKED_ANGLE_DEFAULT 120 // Default servo angle when unlocked
#define WIFI_RESET_HOLD_TIME 3000   // Hold reset button for 3 seconds

// --- Global Variables ---
WebServer server(80);
Servo doorServo;
int servoLockedAngle = SERVO_LOCKED_ANGLE_DEFAULT;
int servoUnlockedAngle = SERVO_UNLOCKED_ANGLE_DEFAULT;
unsigned long unlockTime = 0;
bool isLocked = true;
Preferences preferences;
String savedSSID = "";
String savedPassword = "";
bool isAPMode = false;
unsigned long resetButtonPressTime = 0;
bool resetButtonPressed = false;

// BLE Globals
int rssiThreshold = DEFAULT_RSSI_THRESHOLD;
std::vector<String> authorizedDevices;
// No pBLEScan pointer needed for ArduinoBLE, checking availability differently
unsigned long lastBLEScanTime = 0;
unsigned long scanStartTime = 0;
bool isScanning = false;

// --- Function Prototypes ---
void handleRoot();
void handleUnlock();
void handleLock();
void handleNotFound();
void handleSetup();
void handleSaveWiFi();
void handleReset();
void handleBLESettings();
void handleBLEScan();
void handleAuthorize();
void handleDeauthorize();
void handleSetRSSI();
void handleUpdateServo();
void toggleLock(bool locked);
void setupWiFi();
void startAPMode();
void setupBLE();
void loadBLESettings();
void saveBLESettings();
void loadServoSettings();
void saveServoSettings();
void periodicBLEScan();

// --- Implementation ---

void toggleLock(bool locked) {
  isLocked = locked;
  // Control Servo
  int servoAngle = locked ? servoLockedAngle : servoUnlockedAngle;
  doorServo.write(servoAngle);

  // Control LED (LED on when unlocked for visibility)
  digitalWrite(PIN_STATUS_LED, !locked ? HIGH : LOW);

  Serial.print(locked ? "Door LOCKED" : "Door UNLOCKED");
  Serial.print(" - Servo angle: ");
  Serial.println(servoAngle);
}

void setupWiFi() {
  // Try to load saved credentials
  preferences.begin("wifi-config", false);
  savedSSID = preferences.getString("ssid", "");
  savedPassword = preferences.getString("password", "");
  preferences.end();

  if (savedSSID.length() > 0) {
    Serial.print("Connecting to saved WiFi: ");
    Serial.println(savedSSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(savedSSID.c_str(), savedPassword.c_str());

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
      delay(500);
      Serial.print(".");
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      isAPMode = false;
      // Fast blink to indicate success
      for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(100);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(100);
      }
      digitalWrite(PIN_STATUS_LED, LOW);
      return;
    } else {
      Serial.println("\nFailed to connect to saved WiFi!");
    }
  } else {
    Serial.println("No saved WiFi credentials found.");
  }

  // If we get here, start AP mode
  startAPMode();
}

void startAPMode() {
  Serial.println("Starting Access Point mode...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.println("Connect to WiFi: " + String(AP_SSID));
  Serial.println("Password: " + String(AP_PASSWORD));
  Serial.println("Then go to http://" + IP.toString());

  isAPMode = true;
  // Slow blink to indicate AP mode
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(300);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(300);
  }
}

void handleRoot() {
  if (isAPMode) {
    // Show WiFi setup page in AP mode
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>SmartLock WiFi Setup</title>";
    html +=
        "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body { font-family: sans-serif; max-width: 400px; margin: 50px "
            "auto; padding: 20px; }";
    html += "input { width: 100%; padding: 12px; margin: 8px 0; box-sizing: "
            "border-box; }";
    html += ".button { background-color: #4CAF50; border: none; color: white; "
            "padding: 16px; ";
    html += "width: 100%; text-align: center; font-size: 16px; cursor: "
            "pointer; border-radius: 4px; margin-top: 10px;}";
    html += "h1 { color: #333; }";
    html += "</style></head><body>";
    html += "<h1>SmartLock Setup</h1>";
    html += "<p>Enter your WiFi credentials:</p>";
    html += "<form action='/savewifi' method='POST'>";
    html += "<label>WiFi Network:</label>";
    html += "<input type='text' name='ssid' placeholder='WiFi Name' required>";
    html += "<label>Password:</label>";
    html += "<input type='password' name='password' placeholder='WiFi "
            "Password' required>";
    html += "<button class='button' type='submit'>Save & Connect</button>";
    html += "</form>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  } else {
    // Normal operation mode - show lock control
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>ESP32 Smart Lock</title>";
    html +=
        "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body { font-family: sans-serif; text-align: center; margin-top: "
            "50px; }";
    html += ".button { background-color: #4CAF50; border: none; color: white; "
            "padding: 16px 32px; ";
    html += "text-align: center; text-decoration: none; display: inline-block; "
            "font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: "
            "4px;}";
    html += ".reset { background-color: #f44336; padding: 8px 16px; font-size: "
            "12px; }";
    html += ".status { font-size: 24px; margin-bottom: 20px; }";
    html += ".info { font-size: 12px; color: #666; margin-top: 30px; }";
    html += "</style></head><body>";

    html += "<h1>Control Panel</h1>";
    html += "<div class='status'>Status: " +
            String(isLocked ? "LOCKED" : "UNLOCKED") + "</div>";

    if (isLocked) {
      html +=
          "<a href='/unlock'><button class='button'>UNLOCK DOOR</button></a>";
    } else {
      html += "<a href='/lock'><button class='button' "
              "style='background-color:#FF9800'>LOCK DOOR</button></a>";
    }

    html += "<div class='info'>Connected to: " + savedSSID + "<br>";
    html += "IP: " + WiFi.localIP().toString() + "<br>";
    html += "<h3>Servo Configuration</h3>";
    html += "<form action='/updateservo' method='POST' style='display: "
            "inline-block; text-align: left; background: #f9f9f9; padding: "
            "20px; border-radius: 8px; border: 1px solid #eee;'>";
    html += "<div style='margin-bottom: 15px;'>";
    html += "<label style='display: inline-block; width: 140px;'>Locked "
            "Angle:</label>";
    html += "<input type='number' name='locked' min='0' max='180' "
            "style='width: 60px; padding: 5px;' value='" +
            String(servoLockedAngle) + "'>";
    html += "</div>";
    html += "<div style='margin-bottom: 15px;'>";
    html += "<label style='display: inline-block; width: 140px;'>Unlocked "
            "Angle:</label>";
    html += "<input type='number' name='unlocked' min='0' max='180' "
            "style='width: 60px; padding: 5px;' value='" +
            String(servoUnlockedAngle) + "'>";
    html += "</div>";
    html += "<div style='text-align: center;'>";
    html += "<button class='button' type='submit' style='padding: 10px 20px; "
            "font-size: 14px; margin-top: 5px;'>Update Angles</button>";
    html += "</div>";
    html += "</form><br><br>";
    html += "<a href='/blesettings'><button class='button' "
            "style='background-color:#007bff'>BLE Settings</button></a><br>";
    html += "<a href='/reset'><button class='button reset'>Reset WiFi "
            "Settings</button></a>";
    html += "</div>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  }
}

void handleUnlock() {
  if (!isAPMode) {     // Only allow unlock in normal mode
    toggleLock(false); // Unlock
    unlockTime = millis();
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleLock() {
  if (!isAPMode) {
    toggleLock(true); // Lock
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSaveWiFi() {
  if (!isAPMode) {
    server.send(403, "text/plain", "Not in setup mode");
    return;
  }

  String newSSID = server.arg("ssid");
  String newPassword = server.arg("password");

  if (newSSID.length() > 0) {
    // Save credentials
    preferences.begin("wifi-config", false);
    preferences.putString("ssid", newSSID);
    preferences.putString("password", newPassword);
    preferences.end();

    String html = "<!DOCTYPE html><html><head>";
    html += "<meta http-equiv='refresh' content='10;url=/'>";
    html += "<style>body { font-family: sans-serif; text-align: center; "
            "margin-top: 50px; }</style>";
    html += "</head><body>";
    html += "<h1>Settings Saved!</h1>";
    html += "<p>Connecting to: " + newSSID + "</p>";
    html += "<p>Device will restart in 10 seconds...</p>";
    html += "<p>Please reconnect to your WiFi network and access the device at "
            "its new IP address.</p>";
    html += "</body></html>";
    server.send(200, "text/html", html);

    delay(2000);
    ESP.restart(); // Restart to connect with new credentials
  } else {
    server.send(400, "text/plain", "SSID cannot be empty");
  }
}

void handleReset() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta http-equiv='refresh' content='5;url=/'>";
  html += "<style>body { font-family: sans-serif; text-align: center; "
          "margin-top: 50px; }</style>";
  html += "</head><body>";
  html += "<h1>Resetting WiFi Settings</h1>";
  html += "<p>Device will restart in AP mode...</p>";
  html += "<p>Connect to: " + String(AP_SSID) + "</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);

  // Clear saved credentials
  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();

  delay(2000);
  ESP.restart();
}

void handleNotFound() { server.send(404, "text/plain", "Not found"); }

// --- BLE Implementation ---

void loadBLESettings() {
  preferences.begin("ble-config", true); // Read only
  rssiThreshold = preferences.getInt("rssi", DEFAULT_RSSI_THRESHOLD);
  String devs = preferences.getString("devs", "");
  preferences.end();

  authorizedDevices.clear();
  int start = 0;
  int end = devs.indexOf(';');
  while (end != -1) {
    authorizedDevices.push_back(devs.substring(start, end));
    start = end + 1;
    end = devs.indexOf(';', start);
  }
  if (start < devs.length() && devs.substring(start).length() > 0) {
    authorizedDevices.push_back(devs.substring(start));
  }
}

void saveBLESettings() {
  String devs = "";
  for (String s : authorizedDevices) {
    if (devs.length() > 0)
      devs += ";";
    devs += s;
  }

  preferences.begin("ble-config", false);
  preferences.putString("devs", devs);
  preferences.putInt("rssi", rssiThreshold);
  preferences.end();
}

void setupBLE() {
  loadBLESettings();
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
  }
}

void handleBLESettings() {
  if (isAPMode) {
    server.send(403, "text/plain", "Not available in AP mode");
    return;
  }

  String html = "<!DOCTYPE html><html><head>";
  html += "<title>BLE Settings</title>";
  html +=
      "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: sans-serif; max-width: 600px; margin: 20px "
          "auto; padding: 20px; }";
  html += ".button { background-color: #007bff; border: none; color: white; "
          "padding: 10px 20px; text-decoration: none; border-radius: 4px; "
          "cursor: pointer; }";
  html += ".remove { background-color: #dc3545; font-size: 12px; padding: 5px "
          "10px; margin-left: 10px;}";
  html += "ul { list-style-type: none; padding: 0; }";
  html += "li { padding: 10px; background: #f8f9fa; margin: 5px 0; border: 1px "
          "solid #ddd; display: flex; justify-content: space-between; "
          "align-items: center;}";
  html += "</style></head><body>";

  html += "<h1>BLE Configuration</h1>";
  html += "<a href='/'><button class='button' "
          "style='background-color:#6c757d'>&larr; Back</button></a>";

  html += "<h3>RSSI Threshold: " + String(rssiThreshold) + " dBm</h3>";
  html += "<form action='/setrssi' method='GET'>";
  html += "<input type='range' name='val' min='-100' max='-40' value='" +
          String(rssiThreshold) + "' onchange='this.form.submit()'>";
  html += "</form>";

  html += "<h3>Authorized Devices</h3>";
  html += "<ul>";
  for (String s : authorizedDevices) {
    html += "<li>" + s + " <a href='/deauthorize?addr=" + s +
            "'><button class='button remove'>Remove</button></a></li>";
  }
  html += "</ul>";

  html += "<h3>Scan New Devices</h3>";
  html += "<a href='/blescan'><button class='button'>Scan Nearby "
          "Devices</button></a>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleBLEScan() {
  if (isAPMode) {
    server.send(403, "text/plain", "Not available in AP mode");
    return;
  }

  // Perform Scan
  isScanning = true;
  BLE.scan();

  // We need to collect results over a duration
  struct DiscoveredDevice {
    String address;
    String name;
    int rssi;
  };
  std::vector<DiscoveredDevice> foundDevices;

  unsigned long scanStart = millis();
  while (millis() - scanStart < 5000) {
    BLEDevice peripheral = BLE.available();
    if (peripheral) {
      String addr = peripheral.address();
      bool seen = false;
      for (const auto &d : foundDevices) {
        if (d.address.equalsIgnoreCase(addr))
          seen = true;
      }

      if (!seen) {
        DiscoveredDevice d;
        d.address = addr;
        d.name = peripheral.localName();
        d.rssi = peripheral.rssi();
        foundDevices.push_back(d);
      }
    }
  }

  BLE.stopScan();
  isScanning = false;

  // Sort by RSSI descending
  std::sort(foundDevices.begin(), foundDevices.end(),
            [](const DiscoveredDevice &a, const DiscoveredDevice &b) {
              return a.rssi > b.rssi;
            });

  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Scan Results</title>";
  html +=
      "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body { font-family: sans-serif; padding: 20px; } ul { "
          "list-style: none; padding: 0; } li { margin: 10px 0; border-bottom: "
          "1px solid #eee; padding: 10px; }</style>";
  html += "</head><body>";
  html += "<h1>Found Devices</h1>";
  html += "<a href='/blesettings'>Back to Settings</a>";
  html += "<ul>";

  for (const auto &device : foundDevices) {
    String addr = device.address;
    String name = device.name;
    int rssi = device.rssi;

    bool isAuth = false;
    for (String s : authorizedDevices)
      if (s.equalsIgnoreCase(addr))
        isAuth = true;

    html += "<li>";
    html +=
        "<strong>" + (name.length() > 0 ? name : "Unknown") + "</strong><br>";
    html += "Addr: " + addr + " | RSSI: " + String(rssi) + "<br>";
    if (!isAuth) {
      html += "<a href='/authorize?addr=" + addr + "'>Add to Authorized</a>";
    } else {
      html += "<em>Authorized</em>";
    }
    html += "</li>";
  }
  html += "</ul></body></html>";
  server.send(200, "text/html", html);
}

void handleAuthorize() {
  String addr = server.arg("addr");
  if (addr.length() > 0) {
    bool exists = false;
    for (String s : authorizedDevices)
      if (s.equalsIgnoreCase(addr))
        exists = true;
    if (!exists) {
      authorizedDevices.push_back(addr);
      saveBLESettings();
    }
  }
  server.sendHeader("Location", "/blesettings");
  server.send(303);
}

void handleDeauthorize() {
  String addr = server.arg("addr");
  if (addr.length() > 0) {
    for (int i = 0; i < authorizedDevices.size(); i++) {
      if (authorizedDevices[i].equalsIgnoreCase(addr)) {
        authorizedDevices.erase(authorizedDevices.begin() + i);
        break;
      }
    }
    saveBLESettings();
  }
  server.sendHeader("Location", "/blesettings");
  server.send(303);
}

void handleSetRSSI() {
  String val = server.arg("val");
  if (val.length() > 0) {
    rssiThreshold = val.toInt();
    saveBLESettings();
  }
  server.sendHeader("Location", "/blesettings");
  server.send(303);
}

void handleUpdateServo() {
  if (isAPMode) {
    server.send(403, "text/plain", "Not available in AP mode");
    return;
  }

  int newLocked = server.arg("locked").toInt();
  int newUnlocked = server.arg("unlocked").toInt();

  newLocked = std::max(0, std::min(180, newLocked));
  newUnlocked = std::max(0, std::min(180, newUnlocked));

  servoLockedAngle = newLocked;
  servoUnlockedAngle = newUnlocked;
  saveServoSettings();

  // Refresh servo position to reflect updated angle immediately
  doorServo.write(isLocked ? servoLockedAngle : servoUnlockedAngle);

  server.sendHeader("Location", "/");
  server.send(303);
}

void periodicBLEScan() {
  if (isAPMode) {
    if (isScanning) {
      BLE.stopScan();
      isScanning = false;
    }
    return;
  }

  if (authorizedDevices.size() == 0)
    return;

  // State Machine for Non-blocking Scan
  if (!isScanning) {
    // Check if it's time to start scanning
    if (millis() - lastBLEScanTime >= BLE_SCAN_INTERVAL) {
      isScanning = true;
      BLE.scan(); // Start scanning
      scanStartTime = millis();
      // Serial.println("Starting BLE scan...");
    }
  } else {
    // We are currently scanning
    // 1. Process available devices
    BLEDevice peripheral = BLE.available();
    if (peripheral) {
      String addr = peripheral.address();
      // Serial.print("Found: "); Serial.println(addr);

      for (String auth : authorizedDevices) {
        if (auth.equalsIgnoreCase(addr)) {
          if (peripheral.rssi() >= rssiThreshold) {
            // Found our device!
            if (isLocked) {
              Serial.println("Authorized device found: " + auth +
                             " (RSSI: " + String(peripheral.rssi()) + ")");
              Serial.println("Unlocking...");
              toggleLock(false);
            } else {
              // Serial.println("Refreshed unlock timer for: " + auth);
            }
            unlockTime = millis(); // Refresh auto-lock timer

            // Optional: We can choose to keep scanning or restart.
            // Continuing allows finding other devices or updating RSSI if the
            // stack supports duplicate reporting. But for now, let's keep
            // scanning until timeout to be robust.
          }
        }
      }
    }

    // 2. Check for scan timeout
    if (millis() - scanStartTime >= BLE_SCAN_DURATION) {
      BLE.stopScan();
      isScanning = false;
      lastBLEScanTime = millis();
      // Serial.println("Scan cycle finished.");
    }
  }
}

void loadServoSettings() {
  preferences.begin("servo-config", true);
  servoLockedAngle =
      preferences.getInt("locked", SERVO_LOCKED_ANGLE_DEFAULT);
  servoUnlockedAngle =
      preferences.getInt("unlocked", SERVO_UNLOCKED_ANGLE_DEFAULT);
  preferences.end();
}

void saveServoSettings() {
  preferences.begin("servo-config", false);
  preferences.putInt("locked", servoLockedAngle);
  preferences.putInt("unlocked", servoUnlockedAngle);
  preferences.end();
}

void setup() {
  Serial.begin(115200);

  // Setup Pins
  doorServo.attach(PIN_SERVO); // Attach servo to pin
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_OPEN_BUTTON,
          INPUT_PULLUP); // Assuming button pulled to ground when pressed
  pinMode(PIN_WIFI_RESET, INPUT_PULLUP); // WiFi reset button

  loadServoSettings();

  // Initialize Lock State
  toggleLock(true); // Ensure locked at startup

  // Setup WiFi
  setupWiFi();

  // Setup BLE
  setupBLE();

  // Setup Web Server
  server.on("/", handleRoot);
  server.on("/unlock", handleUnlock);
  server.on("/lock", handleLock);
  server.on("/savewifi", HTTP_POST, handleSaveWiFi);
  server.on("/reset", handleReset);
  server.on("/blesettings", handleBLESettings);
  server.on("/blescan", handleBLEScan);
  server.on("/authorize", handleAuthorize);
  server.on("/deauthorize", handleDeauthorize);
  server.on("/setrssi", handleSetRSSI);
  server.on("/updateservo", HTTP_POST, handleUpdateServo);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  periodicBLEScan();

  // WiFi Reset Button Logic (works in any mode)
  if (digitalRead(PIN_WIFI_RESET) == LOW) {
    if (!resetButtonPressed) {
      resetButtonPressed = true;
      resetButtonPressTime = millis();
      Serial.println("Reset button pressed, hold for 3 seconds...");
    }

    // Check if button held long enough
    if (millis() - resetButtonPressTime >= WIFI_RESET_HOLD_TIME) {
      Serial.println("WiFi Reset triggered!");
      // Blink rapidly to indicate reset
      for (int i = 0; i < 10; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(100);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(100);
      }

      // Clear WiFi credentials
      preferences.begin("wifi-config", false);
      preferences.clear();
      preferences.end();

      Serial.println("WiFi settings cleared. Restarting...");
      delay(1000);
      ESP.restart();
    }
  } else {
    resetButtonPressed = false;
  }

  // Only handle lock/unlock in normal WiFi mode, not in AP setup mode
  if (!isAPMode) {
    // Auto-relock logic
    if (!isLocked && (millis() - unlockTime > LOCK_OPEN_DURATION_MS)) {
      toggleLock(true);
    }

    // Physical Button logic (Simple debouncing/handling)
    if (digitalRead(PIN_OPEN_BUTTON) == LOW) {
      bool newState = !isLocked;
      toggleLock(newState);
      if (!newState) {
        unlockTime = millis();
      }
      // Simple debounce
      delay(200);
      while (digitalRead(PIN_OPEN_BUTTON) == LOW) {
        delay(10);
      } // Wait for release
    }
  }
}
