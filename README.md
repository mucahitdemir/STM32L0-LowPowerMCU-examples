# STM32L0-LowPowerMCU-examples
![image](https://user-images.githubusercontent.com/43001724/173546983-dee75793-f381-4e63-9514-854ce17e5187.png)

On these examples, I will also use **cppcheck** testing tool for static code analysis.
Cppcheck is a static analysis tool for C/C++ code. It provides unique code analysis to detect bugs and focuses on detecting undefined behaviour and dangerous coding constructs. The goal is to have very few false positives. Cppcheck is designed to be able to analyze your C/C++ code even if it has non-standard syntax (common in embedded projects). Link:https://sourceforge.net/p/cppcheck/wiki/Home/

## Step motor control with ULN2003A Motor Driver
In the first example, a step motor is controlled by an stm32f4xx discovery. I could not drive the motor with low power MCU due to low ampere I presume.


## BQ76952 Evalution Board (EVM) for Battery Monitors and Balancers
The BQ76952EVM evaluation module (EVM) is a complete evaluation system for the BQ76952, a 3-cell to 16-cell Li-Ion battery monitor integrated circuit. The EVM consists of a BQ76952 circuit module which is used for simple evaluation of the BQ76952 monitor function. 

Features

- Complete evaluation system for the BQ76952 3-cell to 16-cell Li-Ion and Phosphate battery monitor
- Populated circuit module for 16-cell configuration for quick setup
- Communication available with included USB interface adapter or available on 4-pin connector
- Resistor cell simulator for operation with only a power supply
- This circuit module is tested and includes firmware, GUI, demo and getting started guide.

## BMP180 Digital pressure sensor with stm32L0 low power MCU  
Barometric pressure sensors measure the absolute pressure of the air around them. This pressure varies with both the weather and altitude. 
Depending on how you interpret the data, you can monitor changes in the weather, measure altitude, or any other tasks that require an accurate pressure reading.

**Measuring Weather and Altitude**
The BMP180 was designed to accurately measure atmospheric pressure. 
Atmospheric pressure varies with both weather and altitude; you can measure both of these using this sensor. Here's how:

**What is Atmospheric Pressure?**

The definition of pressure is a force "pressing" on an area. A common unit of pressure is pounds per square inch (psi). 
One pound, pressing on one square inch, equals one psi. The SI unit is newtons per square meter, which are called pascals (Pa).

**Determining Altitude**

Since pressure varies with altitude, you can use a pressure sensor to measure altitude (with a few caveats).

The average pressure of the atmosphere at sea level is 1013.25 hPa (or mbar). This drops off to zero as you climb towards the vacuum of space. Because the curve of this drop-off is well understood, you can compute the altitude difference between two pressure measurements (p and p0) by using this equation:
![image](https://user-images.githubusercontent.com/43001724/173246456-0052798c-0f86-4404-82eb-911c306db4cd.png)

**Tips and Tricks**
Things to Watch Out For

Give it the right voltage: The BMP180 will operate on voltages from 1.8V to 3.6V. We recommend operating it at 3.3V. Never connect the "+" header to voltages higher than 3.6V!. Note that it is safe to connect the SCA and SDL pins to an I2C port on a 5V Arduino, as the pullup resistors on the BMP180 board will keep the voltage below 3.6V.

**Give it air**: Remember that the BMP180 needs access to ambient air to measure its pressure, so don't put it in a sealed case. Providing a small vent hole should be adequate.

**But not too much air**: On the other hand, exposure to fast-moving air or wind can cause momentary pressure variations that will affect your readings. Shield the device from strong air currents.

**Keep it cool**: Because an accurate temperature reading is needed to measure the pressure, try not to expose the device to rapid temperature changes, and keep it away from nearby hot parts and other heat sources.

**Keep it dry**: The BMP180 is sensitive to moisture. Don't submerge it or allow it to contact liquid water.

**Don't blind it**: Surprisingly, the silicon within the BMP180 is sensitive to light, which can enter the device through the hole on the top of the chip. For maximum accuracy, shield the chip from ambient light.

Source: Sparkfun bmp180 tutorial

### Acceleration Sensor (ADXL345)  with STM32L0 MCU
The ADXL345 is a small, thin, ultralow power, 3-axis accelerometer with high resolution (13-bit) measurement at up to ±16 g. 
Digital output data is formatted as 16-bit twos complement and is accessible through either a SPI (3- or 4-wire) or I2C digital interface.
Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

ADXL345 can be used for a whole bunch of things for example:
- 3-axis acceleration measurement
- Roll and Pitch measurement
- Single or double tap measurement
![image](https://user-images.githubusercontent.com/43001724/173820295-4a4bcdc8-97b5-4156-93c4-f7d15ccdf26a.png)



### MPU6050 6 Axis Acceleration and Gyro Sensor

### TIMERS: Motor and LED control with PWM

### Analog-to-Digital Conversion (ADC) Multi-channel

The ADC is one of the most expensive electronic components especially when it does have a high sampling rate and high resolution. Therefore, it’s a valuable resource in microcontrollers and different manufacturers provide us (the firmware engineers) with various features so as to make the best use of it.

 **ADC Reading Methods**

  We can read actually configure the ADC module to take samples (conversions) in 3 different ways. Depending on the application types and requirements you can choose the best fit for you.

**1 Polling**

First of which is the polling method, in this method we’d start an ADC conversion and stop the CPU at this point to wait for the ADC conversion completion. Only after ADC conversion completed, the CPU can resume the main code execution.

**2 Interrupts**

The second method is by using interrupts, so we can trigger the ADC in order to start a conversion and the CPU continues executing the main code routine. Upon conversion completion, the ADC fires an interrupt and the CPU is notified so that it can switch the context to the ISR handler and save the ADC conversion results.

Despite being an efficient way, the interrupt method can add so much overhead to the CPU and cause very high CPU loading. Especially when you’re doing so many conversions per second.


**Direct Memory Access (DMA) Method**

Lastly, the DMA method is the most efficient way of converting multiple ADC channels at very high rates and still transfers the results to the memory without CPU intervention which is so cool and time-saving technique.

### RC522 RFID NFC Module
The RC522 is a 13.56MHz RFID module that is based on the MFRC522 controller from NXP semiconductors. The module can supports I2C, SPI and UART and normally is shipped with a RFID card and key fob. It is commonly used in attendance systems and other person/object identification applications.

![image](https://user-images.githubusercontent.com/43001724/174342020-112e86fd-f7bc-41ac-bcd5-fb7494be6c30.png)

I used this module for authentication for accessing a specific place or program.
### Data storage and LED control (ESP8266 & STM32LOxx)
The ESP8266 WiFi Module is a self contained SOC with integrated TCP/IP protocol stack that can give any microcontroller access to your WiFi network. 
The ESP8266 is capable of either hosting an application or offloading all Wi-Fi networking functions from another application processor.

