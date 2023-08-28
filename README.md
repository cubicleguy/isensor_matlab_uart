# Disclaimer
--------------
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <https://unlicense.org>

# Purpose of this software
-----------------------
This is demonstration software for evaluation purpose to communicate using Matlab with a supported Epson sensor connected to a serial port on a PC.
The user is expected to modify the scripts and extend the functionality to meet their specific evaluation requirements.


# Test machine
--------------

* WIN10 Pro 64-bit/PC Core i5-6500 @ 3.2GHz, 16GB RAM
* Matlab R2018a (9.4.0.813654) 64-bit
* Epson M-G32EV041 USB EvalBoard connected to PC USB (USB-UART)
  * Epson IMU M-G3xx & Epson Accel M-A352AD10

# Requirements
--------------

* This software is tested on MATLAB R2018a or higher [web link](https://www.mathworks.com/)
* This is only supported using UART connection to the Epson Sensor [web link](https://global.epson.com/products_and_drivers/sensing_system/)


# Important for UART Interface
-----------------------------

1. This software makes use of Matlab's serial port interface to communicate between the PC and Epson Sensor.

   When connecting to the PC using a USB Serial Port converter, set the BM Latency option to 1 msec.

   Device Manager -> USB Serial Port (COMx) -> Port Settings -> Advanced
   Change the setting of the Latency Time (msec) in BM Options from *16* (default) to *1*

# How to use this software
-----------------------

1. Create a new object using the Epson Sensor model connected on the serial port from within Matlab.
   Specify the COM port and baudrate (if not 460800) as
   per your hardware configuration.

   For example:

   ```
   >> e = G365Imu('COM12', 460800);
   Issue Software Reset
   Sensor device detected
   Check NOT_READY
   No Errors Detected
   Model: G365PDF0
   Version: 40.19
   Serial#: 00000025
   ```

   The above will create handle object "e" representing the Epson Sensor
   connected to the PC serial port interface.
   A software reset is always asserted to place the Epson Sensor in a known state.
   Then the model number, version code, and serial number is read from the device.

2. Configure the device's burst output fields by writing a 1
   to desired object properties to enable the specific burst fields.

   #### For IMU Devices
   NOTE: Some IMU fields support 32-bit output which can be set by writing a 2

   Object Property | Description
   ----------------|------------
   Ndflag          | Enable NDFLAG field in burst sample, 16-bit only
   TempC           | Enable TEMPC field in burst sample 1=16-bit 2=32-bit
   Gyro            | Enable or disable Gyro X,Y,Z 1=16bit 2=32-bit
   Accl            | Enable or disable Accl X,Y,Z 1=16bit 2=32-bit
   DeltaA          | Enable or disable Delta Angle X,Y,Z 1=16bit 2=32-bit
   DeltaV          | Enable or disable Delta Velocity X,Y,Z 1=16bit 2=32-bit
   Quaternion      | *(Only for G330, G366, G365)* Enable or disable Quaternion q0,q1,q2, q3  1=16bit 2=32-bit
   Atti            | *(Only for G330, G366, G365)* Enable or disable Attitude ANG1, ANG2, ANG3 1=16bit 2=32-bit
   Gpio            | Enable GPIO field in burst sample 16-bit only
   Counter         | Enable 16-bit COUNT field in burst sample
   Chksm16         | Enable 16-bit CHKSM field in burst sample
   Set16G          | * (Only for G330, G366, G370PDG0, G370PDT0) Set accelerometer range 0=8G 1=16G

   #### For Accelerometer Devices:

   Object Property | Description
   ----------------|------------
   Ndflag          | Enable 16-bit NDFLAG field in burst sample
   TempC           | Enable 32-bit TEMPC field in burst sample
   Counter         | Enable 16-bit COUNT field in burst sample
   Chksm16         | Enable 16-bit CHKSM field in burst sample
   TiltX           | Enable tilt output on X axis instead of accelerometer by setting to 1
   TiltY           | Enable tilt output on Y axis instead of accelerometer by setting to 1
   TiltZ           | Enable tilt output on Z axis instead of accelerometer by setting to 1

3. Select the desired output rate & filter setting by setting the object
   properties.

   #### For IMU Devices

   Object Property                                                                          | Description
   -----------------------------------------------------------------------------------------|------------
   DoutRate                                                                                 | 2000, 1000, 500, 250, 125, 62.5, 31.25, 15.625, 400, 200, 100, 80, 50, 40, 25, 20
   FilterSel (All IMUs, and for G370PDF1 & G370PDS0 when DoutRate=2000, 400, or 80sps)      | tap0, tap2, tap4, tap8, tap16, tap32, tap64, tap128, 32fc50, 32fc100, 32fc200, 32fc400, 64fc50, 64fc100, 64fc200, 64fc400, 128fc50, 128fc100, 128fc200, 128fc400
   FilterSel (Only for G370PDF1 & G370PDS0 when DoutRate is not 2000, 400, or 80sps)        | tap0, tap2, tap4, tap8, tap16, tap32, tap64, tap128, 32fc25, 32fc50, 32fc100, 32fc200, 64fc25, 64fc50, 64fc100, 64fc200, 128fc25, 128fc50, 128fc100, 128fc200

   NOTE: Refer to device datasheet for valid combinations of Output
         Rate vs Filter Selection
         i.e. For M-G3xx, Table 5.4 Supported Settings For Output Rate and Filter
              Cutoff Frequency

   #### For Accelerometer Devices

   Object Property      | Description
   ---------------------|------------
   DoutRate             | 1000, 500, 200, 100, 50
   FilterSel            | 64fc83, 64fc220, 128fc36, 128fc110, 128fc350, 512fc9, 512fc16, 512fc60, 512fc210, 512fc460

   NOTE: Refer to device datasheet for valid combinations of Output
         Rate vs Filter Selection
         i.e. For M-A352AD, Table 5.4 Supported Settings For Output Rate and Filter
              Cutoff Frequency

4. Optionally enable other operating modes

   #### For IMU Devices

   Object Property      | Description
   ---------------------|------------
   Gpio2Sel             | GPIO2 pin function can be gpio, counter_reset, or external_trigger
   UartAuto             | Enable UART_AUTO mode which sends burst sampling data automatically

   NOTE: Register read access is not supported when UartAuto is enabled and device is in SAMPLING mode


   #### For Accelerometer Devices

   Object Property     | Description
   --------------------|------------
   ReducedNoiseEn      | Reduce Noise Floor mode
   TempStabEn          | Bias stabilization against thermal shock
   ExtEnable           | External Trigger mode on falling edge
   UartAuto            | Enable UART_AUTO mode which sends burst sampling data automatically

5. To initialize the Epson sensor with current object properties
   run the setDeviceCfg() method.

   *This method should be called after any changes to the object
   properties to propagate the changes to the sensor device.*

```
>> e.setDeviceCfg;
```

6. To retrieve scaled sensor data and store it in an array call the
   getScaledSamples(n) method while specifying the "n" as the number
   of sensor samples to read.

For example to store 500 samples to the scaleddata structure:

```
>>scaleddata = e.getScaledSamples(500);

```

   The above command will store the incoming scaled (converted) sensor data
   into an array while managing the UART interface as follows:

   - If not already in SAMPLING, place the device into SAMPLING mode
   - If UART_AUTO enabled, send Burst Command
   - Burst Read Sensor Data (n samples)
   - Post processes the captured sensor data by applying the
     appropriate scale factors

   The default settings unless changed:

   #### For IMU Devices

   - Output date rate = 250Hz
   - Filter = Moving Average TAP=16
   - Burst Output = Ndflag, 16-bit TempC, 16-bit Gyro X,Y,Z, 16-bit Accl X,Y,Z, GPIO, 16-bit Sample Counter
   - UartAuto is enabled

   #### For Accelerometer Devices

   - Output date rate = 200Hz
   - Filter Tap = 512 TAP Fcutoff = 60Hz
   - Burst Output = 32-bit TempC, 32-bit ACCL X,Y,Z, 16-bit Sample Counter
   - UartAuto is enabled

   ### Precautions For UART Processing Speed

   Due variability of the PC processing latency between Matlab, serialport, Epson Sensor, the sensor output rate may be limited.

   Data output rate above 500Hz may not be stable on your PC.

   If IMU sensor data can not be processed within data output rate timing,
   DRDY toggling will not be consistent and captured sensor data may
   have dropped sensor samples. Verify the processing speed/data rate by
   monitoring the DRDY toggle rate by an oscilloscope probe which should be
   equal to the specified DoutRate.

# Epson UART Device Object Description
---------------------------------

* Object properties must be propagated to the physical device register by calling
  the setDeviceCfg() method after properties are configured.
* To back propagate device register settings to the object properties call
  the getDeviceCfg() method


## Properties For IMU Devices
-------------

Property        | Description
----------------|------------
Accl            | Enable or disable Accl X,Y,Z 1=16bit 2=32-bit
Atti            | *(Only for G365, G330, G366)* Enable or disable Attitude ANG1, ANG2, ANG3 1=16bit 2=32-bit
BaudRate        | Serial Baudrate
BytesPerBurst   | Status indicating # of bytes per burst sample
Chksm16         | Enable 16-bit CHKSM field in burst sample
ComPort         | Serial Port
Counter         | Enable 16-bit COUNT field in burst sample
DeltaA          | Enable or disable Delta Angle X,Y,Z 1=16bit 2=32-bit
DeltaV          | Enable or disable Delta Velocity X,Y,Z 1=16bit 2=32-bit
DoutRate        | Output rate in Hz
ExtEnable       | Enable External Trigger
FieldsInBurst   | Status indicating fields in burst sample
FieldsPerBurst  | Status indicating # of fields per burst sample
FilterSel       | Filter selection
Gpio            | Enable GPIO field in burst sample
Gpio2Sel        | GPIO2 pin function can be gpio, counter_reset, or external_trigger
Gyro            | Enable or disable Gyro X,Y,Z 1=16bit 2=32-bit
Ndflag          | Enable NDFLAG field in burst sample
ProcessSamples  | nsamples to buffer for post processing
Quaternion      | *(Only for G365, G330, G366)* Enable or disable Quaternion q0,q1,q2,q3 1=16bit 2=32-bit
Sampling        | Status indicating SAMPLING mode
TempC           | Enable TEMPC field in burst sample 1=16-bit 2=32-bit
UartAuto        | Enable UART_AUTO mode (Register reading not supported when in SAMPLING mode and UART_AUTO is enabled
Set16G           | *(Only for G330, G366)* 0=8G 1=16G accelerometer output range
IsPDC1          | *(Only for G365)* 0=G365PDF1 1=G365PDC1
IsPDCA          | *(Only for G364)* 0=G364PDC0 1=G364PDCA
IsPDS0          | *(Only for G370PDF1, G370PDS0)* 0=G370PDF1 1=G370PDS0
IsPDT0          | *(Only for G370PDG0, G370PDT0)* 0=G370PDG0 1=G370PDT0
ProdId          | Detected device Product ID from reading device registers
Version         | Detected device Version from reading device registers
SerialId        | Detected device Serial Number from reading device registers
ser             | Serial object (before Matlab 2020a) or serialport object (Matlab 2020a or later)


## Properties For Accelerometer Devices
-------------

Property        | Description
----------------|------------
BaudRate        | Serial Baudrate
BytesPerBurst   | Status indicating # of bytes per burst sample
Chksm16         | Enable 16-bit CHKSM field in burst sample
ComPort         | Serial Port
Counter         | Enable 16-bit COUNT field in burst sample
DoutRate        | Output rate in Hz
ExtEnable       | Enable External Trigger
FieldsInBurst   | Status indicating fields in burst sample
FieldsPerBurst  | Status indicating # of fields per burst sample
FilterSel       | Filter selection
Ndflag          | Enable NDFLAG field in burst sample
ProcessSamples  | nsamples to buffer for post processing
ReducedNoiseEn  | Enable Reduce Noise Floor mode
Sampling        | Status indicating SAMPLING mode
TempC           | Enable TEMPC field in burst sample
TempStabEn      | Enable Bias Stabilization Against Temperature mode
TiltX           | Enable Tilt for X-axis in burst sample instead of acceleration
TiltY           | Enable Tilt for Y-axis in burst sample instead of acceleration
TiltZ           | Enable Tilt for Z-axis in burst sample instead of acceleration
UartAuto        | Enable UART_AUTO mode (Register reading not supported when in SAMPLING mode and UART_AUTO is enabled
ProdId          | Detected device Product ID from reading device registers
Version         | Detected device Version from reading device registers
SerialId        | Detected device Serial Number from reading device registers
ser             | Serial object (before Matlab 2020a) or serialport object (Matlab 2020a or later)


## Object Methods
----------

* Preferred method to configure the device is by setting the object properties and then calling the setDeviceCfg() method to write properties to the appropriate registers
* However, there are methods to access the device registers directly if needed
* Reading device register values directly by calling readReg() method
* To pass the register address to the readReg(<register address>), simply use the object_name.REG_NAME

For example to read MODE_CTRL register REG[0x02(W0)]:

```
>> e.readReg(e.MODE_CTRL, true);
REG[0x02 (W0)] > 0x0400
```

* Writing to device register values directly by calling the writeRegH() or
  writeRegL() methods to write to the HIGH byte or LOW byte respectively.
* NOTE: After writing directly to device registers, run getDeviceCfg() method to synchronize the object properties with current device register settings

For example to write a 0x01 to the HIGH byte of MODE_CTRL register REG[0x02(W0)]:

```
>> e.writeRegH(e.MODE_CTRL, 1, true);
REG[0x03 (W0)] < 0x01
```

For example to write a 0x06 to the LOW byte of MSC_CTRL register REG[0x02(W1)]:

```
>> e.writeRegL(e.MSC_CTRL, 6, true);
REG[0x02 (W1)] < 0x06
```

### Methods For IMU & Accelerometer Devices

Methods         | Description
----------------|------------
dumpReg         | Read back all register values
getDeviceCfg    | Update object properties by reading device registers
getModel        | Read back MODEL number
getScaledSamples| Call getSampling() and convert data to scaled values
getSerialId     | Read back SERIAL_NUM
getVersion      | Read back VERSION number
gotoConfig      | Go to CONFIG mode
gotoSampling    | Go to SAMPLING mode
printTabular    | Output a table listing with specified scaled sensor data
readReg         | Read a 16-bit value from specified WIN_ID & register address
setDeviceCfg    | If not already goto CONFIG mode and program device registers
swResetDevice   | Issue SW Reset and wait RESET_DELAY
writeRegH       | Write to HIGH byte of specified WIN_ID & register address
writeRegL       | Write to LOW byte of specified WIN_ID & register address


### Additional Methods For IMU Devices

Methods         | Description
----------------|------------
testFlash       | Sets Flash test bit and waits for bit to clear to check result
testSelf        | Sets selftest bit and waits for bit to clear to check result


### Additional Methods For Accelerometer Devices

Methods         | Description
----------------|------------
doSelftests     | Perform device self tests (testSENS, testFlash, testACC, testTEMPC, testVDD)


# File Listing
--------------

File                   | Description
-----------------------|------------
EpsonDeviceUart.m      | Handle object base class for Epson serial sensor device connected by PC serial port controlled by Matlab
A352Accl.m             | Epson A352 Sensor object derived from base class EpsonDeviceUart
G320Imu.m              | Epson G320 Sensor object derived from base class EpsonDeviceUart
G330_G366Imu.m         | Epson G330PDG0/G366PDG0 Sensor object derived from base class EpsonDeviceUart
G354Imu.m              | Epson G354 Sensor object derived from base class EpsonDeviceUart
G364Imu.m              | Epson G364PDCA/PDC0 Sensor object derived from base class EpsonDeviceUart
G365Imu.m              | Epson G365PDF1/PDC1 Sensor object derived from base class EpsonDeviceUart
G370F_G370SImu.m       | Epson G370PDF1/PDS0 Sensor object derived from base class EpsonDeviceUart
G370G_G370TImu.m       | Epson G370PDG0/PDT0 Sensor object derived from base class EpsonDeviceUart
exampleEpsonAccl.m     | Example matlab script creating Epson Accelerometer serialport object, configure, get sensor samples, print sensor data to console, plot
exampleEpsonImu.m      | Example matlab script creating Epson IMU serialport object, configure, get sensor samples, print sensor data to console, plot
createAcclPlot.m       | Plot Accl X,Y,Z axes
createGyroPlot.m       | Plot Gyro X,Y,Z axes
README.md              | This readme file in markdown


# Change Record
--------------

Date       | Version | Description
-----------|---------|------------
2021-08-25 |  v1.0   |  Initial release
2023-01-10 |  v1.01  |  Minor maintenance, add G330/G366 support, fix deltaV scalefactor G365PDC1, formatting
2023-08-17 |  v1.02  |  Minor maintenance, add G370PDG0/G370PDT0 support, minor renaming of IMU properties
