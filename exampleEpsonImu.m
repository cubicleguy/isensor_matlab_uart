% This is free and unencumbered software released into the public domain.

% Anyone is free to copy, modify, publish, use, compile, sell, or
% distribute this software, either in source code form or as a compiled
% binary, for any purpose, commercial or non-commercial, and by any
% means.

% In jurisdictions that recognize copyright laws, the author or authors
% of this software dedicate any and all copyright interest in the
% software to the public domain. We make this dedication for the benefit
% of the public at large and to the detriment of our heirs and
% successors. We intend this dedication to be an overt act of
% relinquishment in perpetuity of all present and future rights to this
% software under copyright law.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
% EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
% IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
% OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
% ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
% OTHER DEALINGS IN THE SOFTWARE.

% For more information, please refer to <https://unlicense.org>

% This is example of creating an Epson IMU object, configuring the device, 
% capturing sensor data, printing the scaled sensor data to console,
% and plotting to graph

% Clear workspace
clear;

% Specify # of samples to capture
NUM_OF_SAMPLES = 1000;

% Create Epson Device Object
% Specify COMPort, Baudrate as needed to connect to device

% Latest models
%%%%%%%%%%%%%%%%
e = G330_G366Imu('COM7', 460800);  % For G330PDG0 or G366PDG0 (comport, baudrate)
%e = G365Imu('COM7', 460800);  % For G365PDF1 (comport, baudrate)
%e = G365Imu('COM7', 460800, 1);  % For G365PDC1 (comport, baudrate, IsPDC1=1)
%e = G370F_G370SImu('COM7', 460800);  % For G370PDF1 (comport, baudrate)
%e = G370F_G370SImu('COM7', 460800, 1);  % For G370PDS0 (comport, baudrate, IsPDS0=1)
%e = G370G_G370TImu('COM7', 460800);  % For G370PDG0 (comport, baudrate)
%e = G370G_G370TImu('COM7', 460800, 1);  % For G370PDT0 (comport, baudrate, IsPDT0=1)

% Legacy models
%%%%%%%%%%%%%%%
%e = G320Imu('COM7', 460800);  % For G320 (comport, baudrate)
%e = G354Imu('COM7', 460800);  % For G354 (comport, baudrate)
%e = G364Imu('COM7', 460800);  % For G364PDC0 (comport, baudrate)
%e = G364Imu('COM7', 460800, 1);  % For G364PDCA (comport, baudrate, IsPDCA=1)

% Configure the Epson IMU settings by modifying properties
% as needed
% i.e. e.Property = value;

% Enable UART_AUTO mode (recommended)
% When UART_AUTO is disabled, sample timing in MATLAB is not consistent
e.UartAuto = 1;

e.Ndflag = 1;   % Enable NDFLAG in the burst
e.TempC = 1;  % Enable TEMPC 16-bit in the burst
e.Gyro = 2;  % Enable GYROXYZ 32-bit in the burst
e.Accl = 2;  % Enable ACCLXYZ 32-bit in the burst
e.DeltaA = 0;  % Disable DLTAXYZ in the burst
e.DeltaV = 0;  % Disable DLTVXYZ in the burst
e.Gpio = 0;  % Disable GPIO in the burst
e.Counter = 1;  % Enable COUNTER in the burst
e.Chksm16 = 1;  % Enable CHKSM16 in the burst
e.DoutRate = 125; % Set output rate at 125Hz
e.FilterSel = 'tap32'; % Set filter moving average tap=32

% Properties only supported by G365, G330, G366
%e.Atti = 2;  % Enable Euler ANG123 32-bit in the burst
%e.Quaternion = 2;  % Enable QTN0123 32-bit in the burst

% Properties only supported by G330, G366, G370PDG0, G370PDT0
%e.Set16G = 1;  % Enable 16G range on accelerometers (A_RANGE)

% Initialize Epson IMU registers with properties
e.setDeviceCfg();

% Capture scaled samples into an array
capture = e.getScaledSamples(NUM_OF_SAMPLES);

% Print Scaled Data in Tabular output
e.printTabular(capture);

%Plot
start_col = 1 + e.Ndflag + (e.TempC ~= 0);
createGyroPlot(capture.samples(:,start_col:start_col+2));
createAcclPlot(capture.samples(:,start_col+3:start_col+5));
%end