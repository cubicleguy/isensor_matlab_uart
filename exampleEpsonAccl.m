% Disclaimer:
% --------------
% THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
% INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
% SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
% IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
% IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.

% This is example of creating an Epson ACCL object, configuring the device, 
% capturing sensor data, printing the scaled sensor data to console,
% and plotting to graph

% Clear workspace
clear;

% Specify # of samples to capture
NUM_OF_SAMPLES = 1000;

% Create Epson Device Object
% Specify COMPort, Baudrate as needed to connect to device
e = A352Accl('COM7', 460800);  % For A352AD10

% Configure the Epson Accelerometer settings by modifying properties
% as needed
% i.e. e.Property = value;
e.Ndflag = 1;   % Enable NDFLAG in the burst
e.TempC = 1;    % Enable TempC in the burst
e.Chksm16 = 1;  % Enable CHKSM16 in the burst
e.ReducedNoiseEn = 1; % Enable Reduced Noise Floor mode
e.DoutRate = 100; % Set output rate at 100Hz
e.FilterSel = '512fc9'; % Set filter Tap=512 Fcutoff=9Hz

% Initialize Epson ACCL registers with properties
e.setDeviceCfg();

% Capture 1000 scaled samples into an array
capture = e.getScaledSamples(NUM_OF_SAMPLES);

% Print Scaled Data in Tabular output
e.printTabular(capture);

%Plot Accl
start_col = 1 + e.Ndflag + e.TempC;
createAcclPlot(capture.samples(:,start_col:start_col+2));
%end