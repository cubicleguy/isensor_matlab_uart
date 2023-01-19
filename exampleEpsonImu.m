% Disclaimer:
% --------------
% THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
% INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
% SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
% IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
% IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.

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
%e = G330_G366Imu('COM7', 460800);  % For G330PDG0 or G366PDG0 (comport, baudrate, is16G)
%e = G330_G366Imu('COM7', 460800, 1);  % For G330PDG0 or G366PDG0 with 16G accelerometer (comport, baudrate, is16G=1) output range
e = G370Imu('COM7', 460800);  % For G370PDF1 (comport, baudrate, isPDS0)
%e = G370Imu('COM7', 460800, 1);  % For G370PDS0 (comport, baudrate, isPDS0=1)

% Legacy models
%%%%%%%%%%%%%%%
%e = G320Imu('COM7', 460800);  % For G320 (comport, baudrate)
%e = G354Imu('COM7', 460800);  % For G354 (comport, baudrate)
%e = G364Imu('COM7', 460800);  % For G364PDC0 (comport, baudrate)
%e = G364Imu('COM7', 460800, 1);  % For G364PDCA (comport, baudrate, isPDCA=1)
%e = G365Imu('COM7', 460800);  % For G365PDF1 (comport, baudrate)
%e = G365Imu('COM7', 460800, 1);  % For G365PDC1 (comport, baudrate, isPDC1=1)

% Configure the Epson IMU settings by modifying properties
% as needed
% i.e. e.Property = value;
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

% Properties only supported by G365Imu
%e.Atti = 0;  % Disable Attitude ANG123 in the burst
%e.Quaternion = 0;  % Disable QTN0123 in the burst

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