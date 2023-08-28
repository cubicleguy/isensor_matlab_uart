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
%
% Epson Device handle class represents basic UART device object
% with properties and methods to communicate with the device connected
% to PC serial interface

classdef EpsonDeviceUart < handle

    properties
        Ndflag (1,1) {mustBeInteger, ... % Enable NDFLAG field in burst sample
            mustBeNonnegative, mustBeLessThan(Ndflag, 2)};
        TempC (1,1) {mustBeInteger, ... % TempC field 0=Disable, 1=Enable 16-bit , 2=Enable 32
            mustBeNonnegative, mustBeLessThan(TempC, 3)};
        Counter (1,1) {mustBeInteger, ... % Enable 16-bit COUNT field in burst sample
            mustBeNonnegative, mustBeLessThan(Counter, 2)};
        Chksm16 (1,1) {mustBeInteger, ... % Enable 16-bit CHKSM field in burst sample
            mustBeNonnegative, mustBeLessThan(Chksm16, 2)};
        UartAuto (1,1) {mustBeInteger, ... % Enable UART_AUTO mode
            mustBeNonnegative, mustBeLessThan(UartAuto, 2)};
    end

    properties(GetAccess = 'public' , SetAccess = 'protected')
        Sampling (1,1) {mustBeInteger, ... % Status indicating SAMPLING mode status
            mustBeNonnegative, mustBeLessThan(Sampling, 2)};
        BytesPerBurst (1,1) {mustBeInteger, ... % Status indicating # of bytes per burst sample
            mustBeNonnegative, mustBeLessThan(BytesPerBurst, 74)};
        FieldsPerBurst (1,1) {mustBeInteger, ... % Status indicating # of fields per burst sample
            mustBeNonnegative, mustBeLessThan(FieldsPerBurst, 21)};
        FieldsInBurst = []; % Status indicating fields in burst sample
        ProdId (1,8) char = 'UNKNOWN '; % Detected Product ID
        Version (1,:) char = '255.255'; % Detected Firmware Version
        SerialId (1,8) char = '01234567'; % Detected Serial ID
        BaudRate (1,1) {mustBeInteger, ... % Serial Baudrate
            mustBeMember(BaudRate, [230400, 460800, 921600])} = 460800;
        ComPort (1,:) char; % Serial Port
        ser; % Serial object to device
        ProcessSamples (1,1) {mustBeInteger, ... % nsamples to process at a time, must multiple of # of samples to capture
            mustBeNonnegative, mustBeLessThan(ProcessSamples, 1000)} = 5;
    end

    properties(GetAccess = 'public', SetAccess = 'private', Hidden=true)

        UseSerialport = false; % Support new serialport object
    end

    properties(Constant, Hidden=true)
        % Register Addresses
        MODE_CTRL = [0, hex2dec('02')];
        DIAG_STAT = [0, hex2dec('04')];
        FLAG = [0, hex2dec('06')];
        COUNT = [0, hex2dec('0A')];
        ID = [0, hex2dec('4C')];

        SIG_CTRL = [1, hex2dec('00')];
        MSC_CTRL = [1, hex2dec('02')];
        SMPL_CTRL = [1, hex2dec('04')];
        FILTER_CTRL = [1, hex2dec('06')];
        UART_CTRL = [1, hex2dec('08')];
        GLOB_CMD = [1, hex2dec('0A')];
        PROD_ID1 = [1, hex2dec('6A')];
        PROD_ID2 = [1, hex2dec('6C')];
        PROD_ID3 = [1, hex2dec('6E')];
        PROD_ID4 = [1, hex2dec('70')];
        VERSION = [1, hex2dec('72')];
        SER_NUM1 = [1, hex2dec('74')];
        SER_NUM2 = [1, hex2dec('76')];
        SER_NUM3 = [1, hex2dec('78')];
        SER_NUM4 = [1, hex2dec('7A')];
        WIN_CTRL = [0, hex2dec('FE')];

        % Delimiters for UART command interface
        B_Head = 128; % Start of Burst Char
        B_Term = 13; % Terminator Char

        % Device Timings
        TSTALL = 25e-6;
        RESET_DELAY = 1;
        GOTO_CONFIG_DELAY = 200e-3;
        NOTREADY_DELAY = 100e-3;
    end

    methods

        function obj = EpsonDeviceUart(comport, baudrate)
        % class constructor
            if exist('comport', 'var')
                obj.ComPort = comport;
            end
            if exist('baudrate', 'var')
                obj.BaudRate = baudrate;
            end
            % Release 2020a introduce new serialport object
            rel_ver = version('-release');
            rel_ver = str2double(rel_ver(1:end-1));
            obj.UseSerialport = (rel_ver > 2019);
            if (obj.UseSerialport)
                obj.ser = serialport(obj.ComPort, obj.BaudRate);
                configureCallback(obj.ser, "off");
            else
                obj.ser = serial(obj.ComPort);
                obj.ser.BaudRate = obj.BaudRate;
                obj.ser.Terminator = ''; % No specified terminator
                % Default is 512 bytes, therefore no need to change
                %obj.ser.InputBufferSize = obj.BytesPerBurst * ... % Input Buffer is 4x Processing Sample size
                %    obj.ProcessSamples * 4;
                obj.ser.BytesAvailableFcnMode = 'byte';
                fopen(obj.ser);
            end
            % SW reset, power-on sequence, get device info
            obj.swResetDevice();
            obj.powerOn();
            obj.getModel();
            obj.getVersion();
            obj.getSerialId();
        end

        function delete(obj)
            if not(obj.UseSerialport)
                % class destructor - closes serial port
                fclose(obj.ser);
            end
        end

        function writeRegH(obj, regArray, writeByte, verbose)
        % Write to HIGH byte of specified WIN_ID & register address
        % address must be an even integer value less than 127
            if ~exist('verbose', 'var')
                verbose = 0;
            end
            win_id = regArray(1);
            % OR the address with 0x80
            reg_addr_ = bitor(uint8(regArray(2)), hex2dec('80'));
            % Set WIN_ID
            if (obj.UseSerialport)
                write(obj.ser, [obj.WIN_CTRL(2) win_id obj.B_Term], ...
                    "uint8");
            else
                fwrite(obj.ser, [obj.WIN_CTRL(2) win_id obj.B_Term]);
            end
            % tSTALL
            pause(obj.TSTALL);
            % Write Byte to Address
            if (obj.UseSerialport)
                write(obj.ser, [reg_addr_+1 writeByte obj.B_Term], ...
                    "uint8");
            else
                fwrite(obj.ser, [reg_addr_+1 writeByte obj.B_Term]);
            end
            % tSTALL
            pause(obj.TSTALL);
            if verbose
                fprintf('REG[0x%02x (W%0x)] < 0x%02x\n', regArray(2)+1,...
                    win_id, writeByte);
            end
        end

        function writeRegL(obj, regArray, writeByte, verbose)
        % Write to LOW byte of specified WIN_ID & register address
        % address must be an even integer value less than 127
            if ~exist('verbose', 'var')
                verbose = 0;
            end
            win_id = regArray(1);
            % OR the address with 0x80
            reg_addr_ = bitor(uint8(regArray(2)), hex2dec('80'));
            % Set WIN_ID
            if (obj.UseSerialport)
                write(obj.ser, [obj.WIN_CTRL(2) win_id obj.B_Term], ...
                    "uint8");
            else
                fwrite(obj.ser, [obj.WIN_CTRL(2) win_id obj.B_Term]);
            end
            % tSTALL
            pause(obj.TSTALL);
            % Write Byte to Address
            if (obj.UseSerialport)
                write(obj.ser, [reg_addr_ writeByte obj.B_Term], ...
                    "uint8");
            else
                fwrite(obj.ser, [reg_addr_ writeByte obj.B_Term]);
            end
            % tSTALL
            pause(obj.TSTALL);
            if verbose
                fprintf('REG[0x%02x (W%0x)] < 0x%02x\n', regArray(2),...
                    win_id, writeByte);
            end
        end

        function retval = readReg(obj, regArray, verbose)
        % Read a 16-bit value from specified WIN_ID & register address
        % address must be an even integer value less than 127
            if ~exist('verbose', 'var')
                verbose = 0;
            end
            win_id = regArray(1);
            % AND the address with 0x7E
            reg_addr_ = bitand(uint8(regArray(2)), hex2dec('7E'));
            % Set WIN_ID
            if (obj.UseSerialport)
                write(obj.ser, [obj.WIN_CTRL(2) win_id obj.B_Term], ...
                    "uint8");
            else
                fwrite(obj.ser, [obj.WIN_CTRL(2) win_id obj.B_Term]);
            end
            % tSTALL
            pause(obj.TSTALL);
            % Read 16-bit from Address
            if (obj.UseSerialport)
                write(obj.ser, [reg_addr_ 0 obj.B_Term], ...
                    "uint8");
            else
                fwrite(obj.ser, [reg_addr_ 0 obj.B_Term]);
            end
            % tSTALL
            pause(obj.TSTALL);
            % Get response
            if (obj.UseSerialport)
                retval = read(obj.ser, 4, 'uint8');
            else
                retval = fread(obj.ser, 4, 'uint8');
            end
            % tSTALL
            pause(obj.TSTALL);
            if verbose
                fprintf('REG[0x%02x (W%0x)] > 0x%02x%02x\n', retval(1),...
                    win_id, retval(2), retval(3));
            end
            if (obj.UseSerialport)
                retval = retval(2:3);
            else
                retval = transpose(retval(2:3));
            end
        end

        function swResetDevice(obj)
        % Issue SW Reset and wait RESET_DELAY
            fprintf('Issue Software Reset\n');
            obj.writeRegL(obj.GLOB_CMD, hex2dec('80'));
            pause(obj.RESET_DELAY);
            valID = obj.readReg(obj.ID);
            if ~isequal(valID, uint8([83 69]))
                fprintf('Cannot communicate with sensor device');
                error('Check hardware connection of the sensor device');
            end
            fprintf('Sensor device detected\n');
        end

        function retval = getModel(obj)
        % Read back MODEL number
            x1 = obj.readReg(obj.PROD_ID1);
            x2 = obj.readReg(obj.PROD_ID2);
            x3 = obj.readReg(obj.PROD_ID3);
            x4 = obj.readReg(obj.PROD_ID4);
            retval = [x1(2) x1(1) x2(2) x2(1) x3(2) x3(1) x4(2) x4(1)];
            fprintf('Model: %s\n', native2unicode(retval, 'US-ASCII'));
            obj.ProdId = native2unicode(retval, 'US-ASCII');
        end

        function retval = getVersion(obj)
        % Read back VERSION number
            x1 = obj.readReg(obj.VERSION);
            retval = x1;
            fprintf('Version: %s.%s\n', string(retval));
            obj.Version = compose("%d.%d", retval);
        end

        function retval = getSerialId(obj)
        % Read back SERIAL_NUM
            x1 = obj.readReg(obj.SER_NUM1);
            x2 = obj.readReg(obj.SER_NUM2);
            x3 = obj.readReg(obj.SER_NUM3);
            x4 = obj.readReg(obj.SER_NUM4);
            retval = [x1(2) x1(1) x2(2) x2(1) x3(2) x3(1) x4(2) x4(1)];
            fprintf('Serial#: %s\n', native2unicode(retval, 'US-ASCII'));
            obj.SerialId = native2unicode(retval, 'US-ASCII');
        end

        function gotoSampling(obj)
        % Go to SAMPLING mode
            obj.writeRegH(obj.MODE_CTRL, 1);
            obj.Sampling = 1;
        end

        function gotoConfig(obj)
        % Go to CONFIG mode
            obj.writeRegH(obj.MODE_CTRL, 2);
            pause(obj.GOTO_CONFIG_DELAY);
            if (obj.UseSerialport)
                flush(obj.ser, "input"); % flush input buffer
            else
                flushinput(obj.ser); % flush input buffer
            end
            obj.Sampling = 0;
        end

    end

    methods(Access = 'protected')
    % These methods are protected, (i.e. only accessible from methods of
    % this class or subclasses).

        function powerOn(obj)
        % Power On sequence returns 1 if no errors
            % Check for NOT_READY bit
            fprintf('Check NOT_READY\n');
            tmprd = 4;
            while (tmprd == 4)
                tmp = obj.readReg(obj.GLOB_CMD);
                tmprd = bitand(tmp(1), 4);
                pause(obj.NOTREADY_DELAY);
            end
            tmp = obj.readReg(obj.GLOB_CMD);
            if bitand(tmp(2), hex2dec('E0')) == 0
                fprintf('No Errors Detected\n');
            else
                error('HARD_ERR\n');
            end
        end

        function retval = isSampling(obj)
        % Read MODE_STAT bit to determine if in SAMPLING or not
        % returns 1 if currently in SAMPLING mode
            retval = 0;
            tmp = obj.readReg(obj.MODE_CTRL);
            if tmp(1) == 0
                retval = 1;
            end
        end

        function burstData = getSamples(obj, nsamples)
        % If not already, go to SAMPLING mode
        % if UART_AUTO is disabled, then send BURST_CMD, for every sample
        % until (input buffer>ProcessingSamples) or nsamples, then
        % append burst data to burstData array
        % if UART_AUTO is enabled, then just wait for
        % (input buffer>ProcessingSamples) or nsamples, then append burst
        % data to burstData array
            if(nargin > 0)
                n = nsamples;
            else
                n = 1;
            end

            % storage array excluding header & delimiter
            burstData = zeros(n, obj.BytesPerBurst-2);

            fprintf('Reading %d samples...\n', n);
            % go to SAMPLING if in CONFIG
            if obj.Sampling == 0
                obj.gotoSampling();
            end

            burst_count = 0;
            prevBytesAvailable = 0;
            while (burst_count < n)
                % when UART_AUTO is disabled, need to send BURST_CMD
                if not(obj.UartAuto)
                    % wait for 1 sample period
                    pause(1/obj.DoutRate);

                    % send BURST CMD
                    if (obj.UseSerialport)
                        write(obj.ser, [obj.B_Head 0 obj.B_Term], ...
                            "uint8");
                    else
                        fwrite(obj.ser, [obj.B_Head 0 obj.B_Term]);
                    end

                    % Wait for input buffer to increase by 1 burst
                    bytes_available = 0;
                    while ((bytes_available - prevBytesAvailable) ...
                            < obj.BytesPerBurst)
                        if (obj.UseSerialport)
                            bytes_available = obj.ser.NumBytesAvailable;
                        else
                            bytes_available = obj.ser.BytesAvailable;
                        end
                        pause(1e-3);
                    end
                    if (obj.UseSerialport)
                        prevBytesAvailable = obj.ser.NumBytesAvailable;
                    else
                        prevBytesAvailable = obj.ser.BytesAvailable;
                    end

                end

                % Service the input buffer when > ProcessingSamples
                if (obj.UseSerialport)
                    bytes_available = obj.ser.NumBytesAvailable;
                else
                    bytes_available = obj.ser.BytesAvailable;
                end

                if (bytes_available >= ...
                        (obj.ProcessSamples * (obj.BytesPerBurst)))
                    % Service input buffer in chunks of ProcessingSamples
                    for i = 1:obj.ProcessSamples
                        if (obj.UseSerialport)
                            singleburst = read(obj.ser, ...
                                obj.BytesPerBurst, 'uint8');
                        else
                            singleburst = transpose(fread(obj.ser, ...
                                obj.BytesPerBurst, 'uint8'));
                        end
                        % exclude the Header & Delimiter Byte
                        burstData((i + burst_count), ...
                            :) = singleburst(2:obj.BytesPerBurst-1);
                    end
                    burst_count = burst_count + obj.ProcessSamples;
                    if (obj.UseSerialport)
                        prevBytesAvailable = obj.ser.NumBytesAvailable;
                    else
                        prevBytesAvailable = obj.ser.BytesAvailable;
                    end
                end
            end
            obj.gotoConfig();
        end
    end

end