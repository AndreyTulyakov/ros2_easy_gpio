# EASY GPIO (ROS 2)

It is one of the simplest library for raw working with GPIO with ROS2. Based on `lgpio` library for linux systems.


## Description 


It service only 3 operations:
- `read_bit` - read digital (boolean) signal value on pin.
- `write_bit` - write didital (boolean) value in pin.
- `write_pwm` - set software Pulse-Width Modulation (`frequency` + `duty`) pin output.

You can configure every pin (input or output, mode flags) only one time (at first access) per ROS node runtime. On futher access operations, passed mode flags will be ignored

**Input** operation support pre-pulling modes (pull up, pull down).


**Output** operations supports OD/OS modes, and signal inversion (see below).


### Supported output flags list:
- `OPEN_SOURCE`
- `OPEN_DRAIN`
- `ACTIVE_LOW` (inversion)


### What is Open Source (OS) / Open Drain (OD) output modes?

Can be explained by this simple electrical schematics for output pin:

```
                 VDD
                  |
        OD    ||--+
     +--/ ---o||     P-MOS-FET
     |        ||--+
IN --+            +----- out
     |        ||--+
     +--/ ----||     N-MOS-FET
        OS    ||--+
                  |
                 GND
```

[Original explanation](https://www.kernel.org/doc/html/v5.2/driver-api/gpio/driver.html#gpio-lines-with-open-drain-source-support)


## Dependencies

Install `python3-lgpio` manually, because `rosdep` don't know about this package for now:
```
sudo apt install python3-lgpio
```

or install latest version manually from:

```
https://github.com/joan2937/lg/tree/master
```


## Usage examples

**Use your GPIO device chip** pin numbering (for example Raspberry PI - broadcom (BCM) chip numbering).

### Run GPIO service node:

 ```bash
ros2 run easy_gpio gpio_node
 ```


### Read examples

1. Read from pin 12 (floating)

    ```bash
    # Example: connect pin to external pulled device or line
    ros2 service call /gpio/read_bit easy_gpio_interfaces/ReadBit "pin: 12"
    ```

2. Read from pin 12 (pre-pulled UP):

    ```bash
    # Example: connect pin button to ground
    ros2 service call /gpio/read_bit easy_gpio_interfaces/ReadBit "{pin: 12, pull: UP}"
    ```

3. Read from pin 12 (pre-pulled DOWN):

    ```bash
    # Example: connect pin button to compatible VCC
    ros2 service call /gpio/read_bit easy_gpio_interfaces/ReadBit "{pin: 12, pull: DOWN}"
    ```

### Write examples

1. Write bit to pin 12:

    ```bash
    # Be sure, that pin not connected to heavy load (< 10-30mA)
    ros2 service call /gpio/write_bit easy_gpio_interfaces/WriteBit "{pin: 12, value: 1}"
    ```

2. Write bit to pin 12 with open drain configured:
    ```bash
    ros2 service call /gpio/write_bit easy_gpio_interfaces/WriteBit "{pin: 12, value: 1, mode: OPEN_DRAIN}"
    ```

3. Write bit to pin 12 with open source configured:
    ```bash
    ros2 service call /gpio/write_bit easy_gpio_interfaces/WriteBit "{pin: 12, value: 1, mode: OPEN_SOURCE}"
    ```

4. Write bit to pin 12 with open source and inverted configured:
    ```bash
    ros2 service call /gpio/write_bit easy_gpio_interfaces/WriteBit "{pin: 12, value: 0, mode: OPEN_SOURCE|ACTIVE_LOW }"
    ```

5. Setup PWM signal on pin 12 with 1/4 duty at 1000hz:
    ```bash
    ros2 service call /gpio/write_pwm easy_gpio_interfaces/WritePWM "{pin: 12, frequency: 1000, duty: 25}"
    ```
