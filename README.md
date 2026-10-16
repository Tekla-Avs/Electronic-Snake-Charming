# Electronic Snake Charming: Robot Cobra
A robotic cobra that mimics real-time snake-charming dynamics through sound reactivity.  
Snake body made of servos translate surrounding audio cues into coordinated slithering movements, creating an interactive snake charming experience.

I used PlatformIO in VScode for the code and 3D printer to print the details to keep servos in place.  
The cobra's head was not modeled by me. It was found on a website that shares free 3D assets.

## Hardware

The project combines:
- ESP32 MicroController
- Microphone Amplifier MAX4466
- Two 15 kg/cm Servos
- Five 2kg/cm MG90S Servos
- Servo Driver PCA9685
- Buck Converter 300W 10A DC-DC Step-down module
- Four Tesla Battery Cells in Series
- On/Off switch


## Software
**Web Control Dashboard**

Upon booting, the microcontroller hosts a local web server, accessible via a link generated in the Serial Monitor. The interface acts as a remote control panel with the following features:

- Microphone Toggle: A button to enable or disable the microphone input, seamlessly switching the robot between autonomous and manual modes.

- Force Sleep/Wake Toggle: A manual override button to instantly command the cobra to sleep (lower its body) or wake up (stand at attention).

- Live Servo Sliders: Seven visual slider lines, representing the robot's seven servos. When the microphone is toggled OFF, users can manually drag the dot on each line to precisely set the angle of any individual servo between 0 and 180 degrees.

**Autonomous Audio-Reactive Mode**

When the microphone is toggled ON, the software relies on real-time audio analysis to simulate a classic snake-charming experience. The robot's behavior is dictated by the following logic:

- Sound-Activated Wake & Sleep: The cobra continuously monitors ambient noise. It automatically wakes up when it detects an audio signal and will smoothly return to its resting sleep state if the sound stops for a predefined period.

- FFT-Driven Dancing: While hearing a continuous sound (like a flute or music), the software utilizes a Fast Fourier Transform (FFT) algorithm. This analyzes the frequency and amplitude of the incoming audio to generate synchronized, fluid movements across the 7 servos, making the cobra "dance" to the music.

- Clap Detection: The audio processing includes sharp transient detection. If the cobra hears a loud, distinct clap while dancing, it instantly freezes in place. A second clap signals it to resume its FFT-driven dancing.

## Pin Configuration

### Servo Motors:

Connected Servo Motors to the Servo Driver's pins from 8 to 14.  
Powerful 15kg/cm servos went to the base of the cobra to hold more weight, so, their pins are 8 and 9.



### Servo Driver:

| PCA9685 |   ESP32   |
|---------|-----------|
|   SCL   |  GPIO 22  |
|   SDA   |  GPIO 21  |
|   VCC   |   3.3V    |
|   GND   |    GND    |

 I used DC/DC converter XL4015E1.  
 **P.S.** It was essential to insert 2.2 mF capacitor right at the output pins of **Buck**, parallely to PCA's V+ and GND. 
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


### Batteries:

I used four Tesla cells in series with each other.  
Inserted a switch at the negative side of the battery output.  
Those outputs go to IN+ and IN- of the buck converter.


### BMS Module:

I used BMS Module 1433 4S Li-on Battery Protection 16.8V 30A

This guide details the connection mapping between the **BMS 1433 4S 30A Protection Board** and a **4S Li-ion (16.8V max) battery pack**.

**Battery Wiring Table**

| BMS Board Pin (Left Side) | Battery Pack Node / Wire (Right Side) | Expected Voltage (Nominal / Max) | Description |
| :--- | :--- | :--- | :--- |
| **B-** | Cell 1 Negative (`-`) | **0.0V (GND)** | Main battery ground connection |
| **B1** | Cell 1 (`+`) / Cell 2 (`-`) | **3.7V / 4.2V** | First balance sense tap |
| **B2** | Cell 2 (`+`) / Cell 3 (`-`) | **7.4V / 8.4V** | Second balance sense tap |
| **B3** | Cell 3 (`+`) / Cell 4 (`-`) | **11.1V / 12.6V** | Third balance sense tap |
| **B+** | Cell 4 Positive (`+`) | **14.8V / 16.8V** | Main battery positive connection |
| **P+** | Load (`+`) & Charger (`+`) | **14.8V / 16.8V** | Output positive |
| **P-** | Load (`-`) & Charger (`-`) | **0.0V (GND)** | Output negative |


**Wiring Connection Sequence**

> **Important:** Always solder sensing wires in order from lowest potential (`0V`) to highest potential (`16.8V`) to avoid damaging the protection ICs.

1. **`B-` Wire:** Connect to the total pack ground (Cell 1 Negative).
2. **`B1` Wire:** Connect to the junction between Cell 1 positive and Cell 2 negative.
3. **`B2` Wire:** Connect to the junction between Cell 2 positive and Cell 3 negative.
4. **`B3` Wire:** Connect to the junction between Cell 3 positive and Cell 4 negative.
5. **`B+` Wire:** Connect to the total pack positive (Cell 4 Positive).
6. **`P+` / `P-` Terminals:** Connect your DC output load and 16.8V lithium charger.


**BMS Activation Note**

If `P+` and `P-` show **0V** after initial assembly, apply a **16.8V CC/CV charger** across `P+` and `P-` for **2–3 seconds** to trigger the onboard MOSFET switch and activate the module.



