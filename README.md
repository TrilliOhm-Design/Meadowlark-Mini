# Meadowlark-Mini

Dual deploy programable flight computer for amature rocketry. Equipped with a beeper, physical interrupt, battery monitoring, three pyro channels, IMU, and Pressure sensor. The goal is to offer a feature rich controller that can fit inside a 29mm chassis with the creature comforts of modern controllers. The system is powered by an ESP32-C6 microcontroller.  

![Banner showing PCB](https://github.com/colinhalebrown/Meadowlark-Mini/blob/main/Documentation/images/banner.jpg)

### Hardware Specs
* 10 to 1200 mbar operating range 
* +/- 2.5 mbar pressure error band
* +/- 0.012 mbar relative pressure accuracy
* +/- 32g accelerometer range
* +/- 2000 dps gyroscope
* embedded temperature compensation
* ~35mA operating consumption 
* Buzzer 80db
* 3 pyro channels
* Stable power supply (3-17V)

### Testing
- [x] Hardware validation
- [x] Firmware Initialized
- [ ] Test Functionallity
        - [x] Indicator light
        - [x] Pyro circuit
        - [x] Buzzer
        - [x] Pressure sensor
        - [x] IMU
        - [ ] Flight logs
        - [ ] HighPower supply
- [ ] Software endurance test
- [ ] Simulated flight test
- [ ] Passenger flight tests (collect data and tune triggers)
- [ ] Flight as main controller
- [ ] Flight as secondary controller

Repeat any tests until the controller passes with reliable behavior.

# Hardware
![PCB size](https://github.com/colinhalebrown/Meadowlark-Mini/blob/main/Documentation/images/IMG_4509.jpeg)

The board is laid out such it has pyro channels on top and bottom. With two pyro channels on one side and an enable pin on the other. On the other side of the board you have a JST PH connector for power input from a 2s battery. 

I also wanted easy mounting so the board is mounted using two M3 screws.
### [Hardware Details](https://github.com/colinhalebrown/Meadowlark-Mini/tree/main/Hardware)

# Software

The board is equipped with WIFI, Bluetooth, LoRa and Zigbee. The ESP32-C6 has a built in antenna and a u.FL plug that the user can switch between. 

### Pinout
| System          | Signal Type    | Label    | GPIO         | Verified |
| --------------- | -------------- | -------- | ------------ | -------- |
| Indicator LED   | Digital Output | D13      | GPIO13       | Y        |
| Buzzer          | Digital Output | A0       | GPIO26       | Y        |
| Battery Monior  | Analog Input   | A1       | AD1          | Y        |
| Pyro 1          | Digital Output | D12      | GPIO12       | Y        |
| Pyro 2          | Digital Output | D3       | GPIO6        | Y        |
| Pyro 3          | Digital Output | D6       |              | Y        |
| Pressure Sensor | I2C            | SDA, SCL | GPIO2, GPIO3 | Y        |
| IMU             | I2C            | SDA, SCL | GPIO2, GPIO3 | Y        |

# Status
Currently the board needs software so it can get tested in flight as a payload.
