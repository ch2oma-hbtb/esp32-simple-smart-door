#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// --- Configuration ---
#define AP_SSID "SmartLock-Setup"
#define AP_PASSWORD "12345678"  // Minimum 8 characters for WPA2

// Hardware Pin Definitions for Seeed Studio XIAO ESP32-C3
// Note: D0=GPIO2, D1=GPIO3, D10=GPIO10, etc.
#define PIN_SERVO       D1   // Connect Servo Signal Here
#define PIN_STATUS_LED  D2   // Connect LED Here (XIAO C3 has no user LED on typical pins, use external)
#define PIN_OPEN_BUTTON D8   // Boot Button on XIAO ESP32-C3
#define PIN_WIFI_RESET  D6   // WiFi Reset Button (short to GND to reset WiFi settings)

// Lock Settings
#define LOCK_OPEN_DURATION_MS 15000 // How long the lock stays open in ms
#define SERVO_LOCKED_ANGLE   0     // Servo angle when locked (degrees)
#define SERVO_UNLOCKED_ANGLE 150    // Servo angle when unlocked (degrees)
#define WIFI_RESET_HOLD_TIME 3000   // Hold reset button for 3 seconds

// --- Global Variables ---
WebServer server(80);
Servo doorServo;
unsigned long unlockTime = 0;
bool isLocked = true;
Preferences preferences;
String savedSSID = "";
String savedPassword = "";
bool isAPMode = false;
unsigned long resetButtonPressTime = 0;
bool resetButtonPressed = false;

// --- Function Prototypes ---
void handleRoot();
void handleUnlock();
void handleNotFound();
void handleSetup();
void handleSaveWiFi();
void handleReset();
void toggleLock(bool locked);
void setupWiFi();
void startAPMode();

// --- Implementation ---

void toggleLock(bool locked) {
    isLocked = locked;
    // Control Servo
    int servoAngle = locked ? SERVO_LOCKED_ANGLE : SERVO_UNLOCKED_ANGLE;
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
            for(int i=0; i<5; i++) {
                digitalWrite(PIN_STATUS_LED, HIGH); delay(100);
                digitalWrite(PIN_STATUS_LED, LOW); delay(100);
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
    for(int i=0; i<3; i++) {
        digitalWrite(PIN_STATUS_LED, HIGH); delay(300);
        digitalWrite(PIN_STATUS_LED, LOW); delay(300);
    }
}

void handleRoot() {
    if (isAPMode) {
        // Show WiFi setup page in AP mode
        String html = "<!DOCTYPE html><html><head>";
        html += "<title>SmartLock WiFi Setup</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>";
        html += "body { font-family: sans-serif; max-width: 400px; margin: 50px auto; padding: 20px; }";
        html += "input { width: 100%; padding: 12px; margin: 8px 0; box-sizing: border-box; }";
        html += ".button { background-color: #4CAF50; border: none; color: white; padding: 16px; ";
        html += "width: 100%; text-align: center; font-size: 16px; cursor: pointer; border-radius: 4px; margin-top: 10px;}";
        html += "h1 { color: #333; }";
        html += "</style></head><body>";
        html += "<h1>SmartLock Setup</h1>";
        html += "<p>Enter your WiFi credentials:</p>";
        html += "<form action='/savewifi' method='POST'>";
        html += "<label>WiFi Network:</label>";
        html += "<input type='text' name='ssid' placeholder='WiFi Name' required>";
        html += "<label>Password:</label>";
        html += "<input type='password' name='password' placeholder='WiFi Password' required>";
        html += "<button class='button' type='submit'>Save & Connect</button>";
        html += "</form>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    } else {
        // Normal operation mode - show lock control
        String html = "<!DOCTYPE html><html><head>";
        html += "<title>ESP32 Smart Lock</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>";
        html += "body { font-family: sans-serif; text-align: center; margin-top: 50px; }";
        html += ".button { background-color: #4CAF50; border: none; color: white; padding: 16px 32px; ";
        html += "text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 4px;}";
        html += ".reset { background-color: #f44336; padding: 8px 16px; font-size: 12px; }";
        html += ".status { font-size: 24px; margin-bottom: 20px; }";
        html += ".info { font-size: 12px; color: #666; margin-top: 30px; }";
        html += "</style></head><body>";
        
        html += "<h1>Control Panel</h1>";
        html += "<div class='status'>Status: " + String(isLocked ? "LOCKED" : "UNLOCKED") + "</div>";
        
        if (isLocked) {
            html += "<a href='/unlock'><button class='button'>UNLOCK DOOR</button></a>";
        } else {
            html += "<p>Door relocks automatically...</p>";
        }
        
        html += "<div class='info'>Connected to: " + savedSSID + "<br>";
        html += "IP: " + WiFi.localIP().toString() + "<br>";
        html += "<a href='/reset'><button class='button reset'>Reset WiFi Settings</button></a>";
        html += "</div>";
        html += "</body></html>";
        server.send(200, "text/html", html);
    }
}

void handleUnlock() {
    if (!isAPMode) {  // Only allow unlock in normal mode
        toggleLock(false); // Unlock
        unlockTime = millis();
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
        html += "<style>body { font-family: sans-serif; text-align: center; margin-top: 50px; }</style>";
        html += "</head><body>";
        html += "<h1>Settings Saved!</h1>";
        html += "<p>Connecting to: " + newSSID + "</p>";
        html += "<p>Device will restart in 10 seconds...</p>";
        html += "<p>Please reconnect to your WiFi network and access the device at its new IP address.</p>";
        html += "</body></html>";
        server.send(200, "text/html", html);
        
        delay(2000);
        ESP.restart();  // Restart to connect with new credentials
    } else {
        server.send(400, "text/plain", "SSID cannot be empty");
    }
}

void handleReset() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta http-equiv='refresh' content='5;url=/'>";
    html += "<style>body { font-family: sans-serif; text-align: center; margin-top: 50px; }</style>";
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

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

void setup() {
    Serial.begin(115200);
    
    // Setup Pins
    doorServo.attach(PIN_SERVO);  // Attach servo to pin
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_OPEN_BUTTON, INPUT_PULLUP); // Assuming button pulled to ground when pressed
    pinMode(PIN_WIFI_RESET, INPUT_PULLUP);  // WiFi reset button
    
    // Initialize Lock State
    toggleLock(true); // Ensure locked at startup
    
    // Setup WiFi
    setupWiFi();
    
    // Setup Web Server
    server.on("/", handleRoot);
    server.on("/unlock", handleUnlock);
    server.on("/savewifi", HTTP_POST, handleSaveWiFi);
    server.on("/reset", handleReset);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    server.handleClient();
    
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
            for(int i=0; i<10; i++) {
                digitalWrite(PIN_STATUS_LED, HIGH); delay(100);
                digitalWrite(PIN_STATUS_LED, LOW); delay(100);
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
            toggleLock(false);
            unlockTime = millis();
            // Simple debounce
            delay(200); 
            while(digitalRead(PIN_OPEN_BUTTON) == LOW) { delay(10); } // Wait for release
        }
    }
}
