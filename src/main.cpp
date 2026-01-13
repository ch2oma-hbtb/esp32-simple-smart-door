#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// --- Configuration ---
const char* ssid = "Pond2";
const char* password = "84383972Aa";

// Hardware Pin Definitions for Seeed Studio XIAO ESP32-C3
// Note: D0=GPIO2, D1=GPIO3, D10=GPIO10, etc.
#define PIN_SERVO       D1   // Connect Servo Signal Here
#define PIN_STATUS_LED  D2   // Connect LED Here (XIAO C3 has no user LED on typical pins, use external)
#define PIN_OPEN_BUTTON D8   // Boot Button on XIAO ESP32-C3

// Lock Settings
#define LOCK_OPEN_DURATION_MS 15000 // How long the lock stays open in ms
#define SERVO_LOCKED_ANGLE   0     // Servo angle when locked (degrees)
#define SERVO_UNLOCKED_ANGLE 150    // Servo angle when unlocked (degrees)

// --- Global Variables ---
WebServer server(80);
Servo doorServo;
unsigned long unlockTime = 0;
bool isLocked = true;

// --- Function Prototypes ---
void handleRoot();
void handleUnlock();
void handleNotFound();
void toggleLock(bool locked);
void setupWiFi();

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
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500);
        Serial.print(".");
        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED)); // Blink while connecting
        retries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected.");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        // Fast blink to indicate success
        for(int i=0; i<5; i++) {
             digitalWrite(PIN_STATUS_LED, HIGH); delay(100);
             digitalWrite(PIN_STATUS_LED, LOW); delay(100);
        }
    } else {
        Serial.println("\nWiFi Connection Failed!");
    }
    digitalWrite(PIN_STATUS_LED, LOW); // LED off default
}

void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<title>ESP32 Smart Lock</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body { font-family: sans-serif; text-align: center; margin-top: 50px; }";
    html += ".button { background-color: #4CAF50; border: none; color: white; padding: 16px 32px; ";
    html += "text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 4px;}";
    html += ".status { font-size: 24px; margin-bottom: 20px; }";
    html += "</style></head><body>";
    
    html += "<h1>Control Panel</h1>";
    html += "<div class='status'>Status: " + String(isLocked ? "LOCKED" : "UNLOCKED") + "</div>";
    
    if (isLocked) {
        html += "<a href='/unlock'><button class='button'>UNLOCK DOOR</button></a>";
    } else {
        html += "<p>Door connects automatically...</p>";
    }
    
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleUnlock() {
    toggleLock(false); // Unlock
    unlockTime = millis();
    server.sendHeader("Location", "/");
    server.send(303);
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
    
    // Initialize Lock State
    toggleLock(true); // Ensure locked at startup
    
    // Setup WiFi
    setupWiFi();
    
    // Setup Web Server
    server.on("/", handleRoot);
    server.on("/unlock", handleUnlock);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    server.handleClient();
    
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
