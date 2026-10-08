# Electronic-Snake-Charming---Robot-Cobra
A robotic cobra that mimics real-time snake-charming dynamics through sound reactivity. Its body consists of multiple servo-driven segments that translate surrounding audio cues into coordinated slithering and swaying movements, creating an interactive, lifelike snake charming experience.

## Overview

## Pinout

### Servo Motors:

Connected Servo Motors to the Servo Driver's pins from 8 to 14.

### Servo Driver:

| PCA9685 |   ESP32   |
|---------|-----------|
|   SCL   |  GPIO 22  |
|   SDA   |  GPIO 21  |
|   VCC   |   3.3V    |
|   GND   |    GND    |

 I used DC/DC converter XL4015E1.
 P.S. It was essential to insert 2.2 mF capacitor right at the output pins, parallely to PCA's V+ and GND.
 Otherwise buck used to shut down.
 
| PCA9685 |   Buck  |
|---------|---------|
|   V+    |   OUT+  |
|   GND   |   OUT-  |

### Microphone Amplifier: MAX4466 -> ESP32

| MAX4466 |   ESP32   |
|---------|-----------|
|   OUT   |  GPIO 34  |
|   VCC   |    3.3V   |
|   GND   |    GND    |

### BMS Module:

I used BMS Module 1433 4S Li-on Battery Protection 16.8V 30A

This guide details the connection mapping between the **BMS 1433 4S 30A Protection Board** and a **4S Li-ion (16.8V max) battery pack**.

## Battery Wiring Table

| BMS Board Pin (Left Side) | Battery Pack Node / Wire (Right Side) | Expected Voltage (Nominal / Max) | Description |
| :--- | :--- | :--- | :--- |
| **B-** | Cell 1 Negative (`-`) | **0.0V (GND)** | Main battery ground connection |
| **B1** | Cell 1 (`+`) / Cell 2 (`-`) | **3.7V / 4.2V** | First balance sense tap |
| **B2** | Cell 2 (`+`) / Cell 3 (`-`) | **7.4V / 8.4V** | Second balance sense tap |
| **B3** | Cell 3 (`+`) / Cell 4 (`-`) | **11.1V / 12.6V** | Third balance sense tap |
| **B+** | Cell 4 Positive (`+`) | **14.8V / 16.8V** | Main battery positive connection |
| **P+** | Load (`+`) & Charger (`+`) | **14.8V / 16.8V** | Output positive / Charge positive |
| **P-** | Load (`-`) & Charger (`-`) | **0.0V (GND)** | Switched output negative / Charge negative |


## Wiring Connection Sequence

> **Important:** Always solder sensing wires in order from lowest potential (`0V`) to highest potential (`16.8V`) to avoid damaging the protection ICs.

1. **`B-` Wire:** Connect to the total pack ground (Cell 1 Negative).
2. **`B1` Wire:** Connect to the junction between Cell 1 positive and Cell 2 negative.
3. **`B2` Wire:** Connect to the junction between Cell 2 positive and Cell 3 negative.
4. **`B3` Wire:** Connect to the junction between Cell 3 positive and Cell 4 negative.
5. **`B+` Wire:** Connect to the total pack positive (Cell 4 Positive).
6. **`P+` / `P-` Terminals:** Connect your DC output load and 16.8V lithium charger.


## BMS Activation Note

If `P+` and `P-` show **0V** after initial assembly, apply a **16.8V CC/CV charger** across `P+` and `P-` for **2–3 seconds** to trigger the onboard MOSFET switch and activate the module.



### Batteries:

I used four Tesla cells in series with each other.
Inserted a switch at the negative side of the battery output.
Those outputs go to IN+ and IN- of the buck converter.



