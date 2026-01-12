# ESP32 Simple Smart Door Lock

This project is a simple firmware for an ESP32-based smart door lock. It provides a web interface to unlock the door connected via a relay.

## Features
- **WiFi Connectivity**: Connects to your home network.
- **Web Interface**: Simple HTML controls to unlock the door.
- **Auto-Lock**: Automatically relocks after 3 seconds.
- **Physical Button**: Support for a physical push button to unlock from inside.
- **Status LED**: Visual feedback for connection status and lock state.

## Hardware Requirements
1. **ESP32 Development Board** (e.g., Seeed Studio XIAO ESP32-C3)
2. **Relay Module** or MOSFET (suitable for your lock voltage, e.g., 12V Solenoid)
3. **Electronic Lock** (Solenoid lock, Magnetic lock, etc.)
4. **Push Button** (Optional, for manual open)
5. **Power Supply** (Appropriate for ESP32 and Lock)

## Wiring / Pin Configuration
You can change these in `src/main.cpp`.

| Component     | GPIO Pin | Seeed XIAO ESP32-C3 Pin | Description |
| ------------- | :---: | :---: | ----------- |
| **Lock Relay** | `GPIO 3` | `D1` | Connects to the IN pin of the relay module. |
| **Status LED** | `GPIO 2` | `D0` | Connect external LED (series 220ohm resistor) to GND. |
| **Open Button**| `GPIO 9` | `BOOT` | The "BOOT" button on the board. Connects to GND when pressed. |

## Installation & Setup

1. **Install PlatformIO**:
   - Install VS Code and the **PlatformIO IDE** extension.

2. **Open Project**:
   - Open this folder in VS Code. PlatformIO should initialize automatically.

3. **Configure WiFi**:
   - Open `src/main.cpp`.
   - Edit the lines:
     ```cpp
     const char* ssid = "YOUR_WIFI_SSID";
     const char* password = "YOUR_WIFI_PASSWORD";
     ```
   - Enter your actual WiFi credentials.

4. **Upload Firmware**:
   - Connect your ESP32 via USB.
   - Click the **PlatformIO: Upload** button (arrow icon in the bottom status bar) or run:
     ```bash
     pio run -t upload
     ```

5. **Monitor & Usage**:
   - Open the Serial Monitor (`pio device monitor`) to see the IP Address assigned to the ESP32.
   - Open that IP Address in a web browser on your phone or computer.
   - Click **UNLOCK DOOR** to trigger the lock.

## Customization
- **Lock Duration**: Change `LOCK_OPEN_DURATION_MS` in `src/main.cpp` to keep the door open longer.
- **Relay Logic**: If your relay triggers on LOW signal (Active Low), change `RELAY_ACTIVE_HIGH` to `false`.
