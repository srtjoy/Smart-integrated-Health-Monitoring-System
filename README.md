# ECG Patient Monitoring System

An ESP32- and AD8232-based prototype for ECG signal acquisition and patient-vital monitoring. The project includes ESP32 firmware, a Python desktop monitoring application, and a SIM800L communication test sketch.

> **Important safety notice:** This repository documents an academic prototype, not a certified medical device. It is not intended for diagnosis, treatment, or emergency decisions. Sensor readings and automated risk labels must not be treated as clinical conclusions. Validate the hardware and algorithms with qualified supervision before any human-use study.

## Project contents

```text
ECG-Patient-Monitoring-System/
├── firmware/
│   ├── health_monitor_v5_1.ino   # Main ESP32 firmware (latest supplied version)
│   └── health_monitor_v5_0.ino   # Earlier firmware version
├── desktop_app/
│   └── ecg_patient_monitor.py    # Python desktop dashboard
├── tools/
│   └── sim800l_test/
│       └── sim800l_test.ino      # SIM800L module test sketch
├── docs/                         # Add thesis, circuit, and setup documents here
├── assets/                       # Add anonymised project images here
├── requirements.txt
└── .gitignore
```

## Main features

- ECG signal acquisition using AD8232 and ESP32
- ECG sampling configured at 200 Hz in the supplied firmware/application
- Wi-Fi WebSocket and serial communication options in the desktop application
- Live ECG waveform and heart-rate display
- Additional sensor integration in the ESP32 firmware, including pulse oximetry, temperature, blood-pressure sensing circuitry, and environmental sensing
- SIM800L-based SMS functionality
- CSV export and screenshot capture in the desktop application

Features and performance depend on the actual hardware configuration, sensor calibration, firmware version, and test conditions.

## Hardware

The supplied firmware comments identify these modules/interfaces:

- ESP32 development board
- AD8232 ECG module
- MAX30100 pulse-oximeter module
- DS18B20 temperature sensor
- DHT sensor (DHT library; configured as DHT22 in the supplied firmware)
- SIM800L GSM module
- I2C LCD (20×4, address `0x27`)
- Blood-pressure sensing circuit and associated components

Check the pin definitions in `firmware/health_monitor_v5_1.ino` and verify your own wiring before powering the circuit. Do not assume every module is present in every hardware revision.

## Desktop application setup

Use Python 3.10 or newer (recommended). From the repository root:

```bash
python -m venv .venv
```

Activate the environment, then install dependencies:

**Windows PowerShell**

```powershell
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python desktop_app/ecg_patient_monitor.py
```

**macOS / Linux**

```bash
source .venv/bin/activate
pip install -r requirements.txt
python desktop_app/ecg_patient_monitor.py
```

The application uses Tkinter and Matplotlib's Tk backend. On some Linux systems, Tkinter must be installed through the operating-system package manager.

## ESP32 firmware setup

1. Install Arduino IDE and the ESP32 board support package.
2. Open `firmware/health_monitor_v5_1.ino`.
3. Install the libraries included by the sketch, matching the APIs used by the source.
4. Select the correct ESP32 board and serial port.
5. Review pin assignments, sensor addresses, SIM800L power requirements, and Wi-Fi configuration.
6. Compile and test each subsystem separately before connecting the complete system.

The supplied firmware uses WiFiManager for Wi-Fi configuration. Confirm the library versions and wiring for your specific board.

## SIM800L test

The isolated test sketch is in `tools/sim800l_test/`. Before using it, edit the test phone number and APN settings locally. Do not commit personal phone numbers, SIM credentials, APN passwords, or other private configuration.

## Data and privacy

Do not upload identifiable patient information, phone numbers, private network details, or raw health records. Use anonymised, consented sample data only. The screenshots supplied during preparation appear to contain patient-identifying labels, so they have intentionally not been included in this starter repository. Add only redacted screenshots to `assets/`.

## Versions

- `health_monitor_v5_1.ino`: latest supplied ESP32 firmware version.
- `health_monitor_v5_0.ino`: earlier firmware version retained for comparison.
- `ecg_patient_monitor.py`: supplied Python desktop application v2.1. The two uploaded Python copies were byte-for-byte identical, so only one copy is included.

## Research and academic use

Add your thesis title, author details, supervisor information (if approved), methodology, circuit diagram, test procedure, results, limitations, and references in `docs/` or in this README. Clearly distinguish measured results from simulated or illustrative values.
