% Disclaimer:
% --------------
% THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
% INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
% SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
% IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
% IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.
%
% Epson G330 & G366 IMU derived from Epson Device Uart handle class

classdef G330_G366Imu < EpsonDeviceUart

    properties
        Gyro (1,1) {mustBeInteger, ... % GyroXYZ field 0=Disable, 1=Enable 16-bit , 2=Enable 32
            mustBeNonnegative, mustBeLessThan(Gyro, 3)} = 1;
        Accl (1,1) {mustBeInteger, ... % AcclXYZ field 0=Disable, 1=Enable 16-bit , 2=Enable 32
            mustBeNonnegative, mustBeLessThan(Accl, 3)} = 1;
        DeltaA (1,1) {mustBeInteger, ... % Delta Angle field 0=Disable, 1=Enable 16-bit , 2=Enable 32
            mustBeNonnegative, mustBeLessThan(DeltaA, 3)} = 0;
        DeltaV (1,1) {mustBeInteger, ... % Delta Velocity field 0=Disable, 1=Enable 16-bit , 2=Enable 32
            mustBeNonnegative, mustBeLessThan(DeltaV, 3)} = 0;
        Quaternion (1,1) {mustBeInteger, ... % Quaternion field q0, q1, q2, q3 0=Disable, 1=Enable 16-bit , 2=Enable 32
            mustBeNonnegative, mustBeLessThan(Quaternion, 3)} = 0;
        Atti (1,1) {mustBeInteger, ... % Attitude field ANG1/ANG2/ANG3 0=Disable, 1=Enable 16-bit , 2=Enable 32
            mustBeNonnegative, mustBeLessThan(Atti, 3)} = 0;
        Gpio (1,1) {mustBeInteger, ... % Enable GPIO field in burst sample
            mustBeNonnegative, mustBeLessThan(Gpio, 2)} = 1;
        DoutRate (1,1) double {mustBeNonnegative, ... % Output rate in Hz
            mustBeLessThanOrEqual(DoutRate, 2000)} = 250;
        FilterSel (1,:) char {mustBeMember(FilterSel,{... % Filter selection
            'tap0', 'tap2', 'tap4', 'tap8', 'tap16',...
            'tap32', 'tap64', 'tap128', ...
            '32fc50', '32fc100', '32fc200', '32fc400',...
            '64fc50', '64fc100', '64fc200', '64fc400',...
            '128fc50', '128fc100', '128fc200', '128fc400'...
             })} = 'tap16';
        Gpio2Sel (1,:) char {mustBeMember(Gpio2Sel,{... % GPIO2 pin function selection NOTE: external trigger assumes using external trigger modeB (modeA is not supported)
            'gpio', 'ext_trigger', 'counter_reset'...
             })} = 'gpio';
        is16G (1,1) {mustBeInteger, ... % 0=8G (default), 1=16G accelerometer output range
            mustBeNonnegative, mustBeLessThan(is16G, 2)} = 0;
    end

    properties(GetAccess = 'public', SetAccess = 'private', Hidden=true)
        % Scalefactors (initially default to G330PDG0 or G366PDG0)
        SF_GYR = 1/66; % Scale factor to convert to deg/sec
        SF_ACC = 1/4; % Scale factor to convert to mG (milli-gravity)
        SF_ANG = 0.00699411; % Scale factor to convert to degrees
        SF_DLTA = 3.103e-2; % Scale factor to convert to deg/bit
        SF_DLTV = 5.021e-3; % Scale factor to convert to (m/s)/bit
        SF_TEMPC = 0.00390625; % Scale factor to convert to degC
    end

    properties(Constant, Hidden=true)
        % Register Addresses
        GPIO = [0, hex2dec('08')];
        RANGE_OVER = [0, hex2dec('0C')];
        TEMP_HIGH = [0, hex2dec('0E')];
        TEMP_LOW = [0, hex2dec('10')];
        XGYRO_HIGH = [0, hex2dec('12')];
        XGYRO_LOW = [0, hex2dec('14')];
        YGYRO_HIGH = [0, hex2dec('16')];
        YGYRO_LOW = [0, hex2dec('18')];
        ZGYRO_HIGH = [0, hex2dec('1A')];
        ZGYRO_LOW = [0, hex2dec('1C')];
        XACCL_HIGH = [0, hex2dec('1E')];
        XACCL_LOW = [0, hex2dec('20')];
        YACCL_HIGH = [0, hex2dec('22')];
        YACCL_LOW = [0, hex2dec('24')];
        ZACCL_HIGH = [0, hex2dec('26')];
        ZACCL_LOW = [0, hex2dec('28')];

        QTN0_HIGH = [0, hex2dec('50')];
        QTN0_LOW = [0, hex2dec('52')];
        QTN1_HIGH = [0, hex2dec('54')];
        QTN1_LOW = [0, hex2dec('56')];
        QTN2_HIGH = [0, hex2dec('58')];
        QTN2_LOW = [0, hex2dec('5A')];
        QTN3_HIGH = [0, hex2dec('5C')];
        QTN3_LOW = [0, hex2dec('5E')];
        XDLTA_HIGH = [0, hex2dec('64')];
        XDLTA_LOW = [0, hex2dec('66')];
        YDLTA_HIGH = [0, hex2dec('68')];
        YDLTA_LOW = [0, hex2dec('6A')];
        ZDLTA_HIGH = [0, hex2dec('6C')];
        ZDLTA_LOW = [0, hex2dec('6E')];
        XDLTV_HIGH = [0, hex2dec('70')];
        XDLTV_LOW = [0, hex2dec('72')];
        YDLTV_HIGH = [0, hex2dec('74')];
        YDLTV_LOW = [0, hex2dec('76')];
        ZDLTV_HIGH = [0, hex2dec('78')];
        ZDLTV_LOW = [0, hex2dec('7A')];

        BURST_CTRL1 = [1, hex2dec('0C')];
        BURST_CTRL2 = [1, hex2dec('0E')];
        POL_CTRL = [1, hex2dec('10')];
        GLOB_CMD3 = [1, hex2dec('12')];
        ATTI_CTRL = [1, hex2dec('14')];
        GLOB_CMD2 = [1, hex2dec('16')];
        R_MATRIX_M11 = [1, hex2dec('38')];
        R_MATRIX_M12 = [1, hex2dec('3A')];
        R_MATRIX_M13 = [1, hex2dec('3C')];
        R_MATRIX_M21 = [1, hex2dec('3E')];
        R_MATRIX_M22 = [1, hex2dec('40')];
        R_MATRIX_M23 = [1, hex2dec('42')];
        R_MATRIX_M31 = [1, hex2dec('44')];
        R_MATRIX_M32 = [1, hex2dec('46')];
        R_MATRIX_M33 = [1, hex2dec('48')];

        % Device Timings
        SELFTEST_DELAY = 80e-3;
        FLASHTEST_DELAY = 30e-3;
    end

    methods

        function obj = G330_G366Imu(comport, baudrate, is16G)
        % class constructor
            if not(exist('comport', 'var'))
                error('Error: Must specify comport');
            end
            if not(exist('baudrate', 'var'))
                error('Error: Must specify baudrate');
            end
            obj@EpsonDeviceUart(comport, baudrate);
            if (exist('is16G', 'var'))
                obj.is16G = is16G;
            end
            obj.getDeviceCfg();
        end

        function getDeviceCfg(obj)
        % Update object properties by reading device registers
        % Ndflag, TempC, Counter, Chksm16
        % Attitude, DoutRate, FilterSel
        % Gpio2, UartAuto
        % BytesPerBurst, FieldsPerBurst, FieldInBurst

            % In UART_AUTO & SAMPLING need to goto CONFIG mode and clear
            % input buffer
            if (obj.Sampling && obj.UartAuto)
                obj.gotoConfig();
            end

            % Read GPIO2 function setting
            mscCtrl = obj.readReg(obj.MSC_CTRL);
            mscCtrlHigh = bitand(mscCtrl(2), hex2dec('C0'));
            obj.decodeExtSel(mscCtrlHigh);

            % Read Output Data Rate
            smplCtrl = obj.readReg(obj.SMPL_CTRL);
            obj.decodeDoutRate(smplCtrl(1));

            % Read Filter Setting
            filterCtrl = obj.readReg(obj.FILTER_CTRL);
            obj.decodeFilterSel(filterCtrl(2));

            % Read UART_CTRL Setting
            uartCtrl = obj.readReg(obj.UART_CTRL);
            obj.UartAuto = logical(bitand(uartCtrl(2), hex2dec('01')));

            % Read Burst Field Setting
            burstCtrl1 = obj.readReg(obj.BURST_CTRL1);
            burstCtrl2 = obj.readReg(obj.BURST_CTRL2);

            % Read GLOB_CMD3 Setting
            globCmd3 = obj.readReg(obj.GLOB_CMD3);

            % NDFLAG
            obj.Ndflag = any(bitand(burstCtrl1(1), hex2dec('80')));

            % TEMPC
            if bitand(burstCtrl1(1), hex2dec('40'))
                if bitand(burstCtrl2(1), hex2dec('40'))
                    obj.TempC = 2;
                else
                    obj.TempC = 1;
                end
            else
                obj.TempC = 0;
            end

            % GYRO
            if bitand(burstCtrl1(1), hex2dec('20'))
                if bitand(burstCtrl2(1), hex2dec('20'))
                    obj.Gyro = 2;
                else
                    obj.Gyro = 1;
                end
            else
                obj.Gyro = 0;
            end

            % ACCL
            if bitand(burstCtrl1(1), hex2dec('10'))
                if bitand(burstCtrl2(1), hex2dec('10'))
                    obj.Accl = 2;
                else
                    obj.Accl = 1;
                end
            else
                obj.Accl = 0;
            end
            % DLTA
            if bitand(burstCtrl1(1), hex2dec('08'))
                if bitand(burstCtrl2(1), hex2dec('08'))
                    obj.DeltaA = 2;
                else
                    obj.DeltaA = 1;
                end
            else
                obj.DeltaA = 0;
            end

            % DLTV
            if bitand(burstCtrl1(1), hex2dec('04'))
                if bitand(burstCtrl2(1), hex2dec('04'))
                    obj.DeltaV = 2;
                else
                    obj.DeltaV = 1;
                end
            else
                obj.DeltaV = 0;
            end

            % QTN
            if bitand(burstCtrl1(1), hex2dec('02'))
                if bitand(burstCtrl2(1), hex2dec('02'))
                    obj.Quaternion = 2;
                else
                    obj.Quaternion = 1;
                end
            else
                obj.Quaternion = 0;
            end

            % ATTI
            if bitand(burstCtrl1(1), hex2dec('01'))
                if bitand(burstCtrl2(1), hex2dec('01'))
                    obj.Atti = 2;
                else
                    obj.Atti = 1;
                end
            else
                obj.Atti = 0;
            end

            % Counter
            obj.Counter = any(bitand(burstCtrl1(2), hex2dec('02')));

            % Chksm16
            obj.Chksm16 = any(bitand(burstCtrl1(2), hex2dec('01')));

            % is16G
            obj.is16G = any(bitand(globCmd3(2), hex2dec('01')));

            obj.updateBytesPerBurst();
            obj.updateFieldsPerBurst();
            obj.updateFieldsInBurst();
        end

        function setDeviceCfg(obj)
        % If not already goto CONFIG mode and program device registers
        % SMPL_CTRL, FILTER_CTRL, MSC_CTRL, BURST_CTRL,
        % SIG_CTRL, UART_CTRL based on object properties
        % Property is16G sets the proper SF_ACC & SF_DTLV (16G vs 8G)
        % Will automatically enter CONFIG mode
            % Register writes need to be in CONFIG mode and flush
            % any data in input buffer
            if obj.Sampling == 1
                obj.gotoConfig();
            end
            obj.setOutputRate();
            obj.setFilter();
            % Restriction when External Trigger is enabled
            if strcmpi(obj.Gpio2Sel, 'external_trigger')
                if (logical(obj.Atti) || logical(obj.Quaternion))
                    fprintf('Error: Atti or Quaternion cannot be enabled ');
                    fprintf('when external_trigger is enabled.');
                    fprintf('Disabling Atti, Quaternion as fallback.');
                    obj.Atti = 0;
                    obj.Quaternion = 0;
                end
            end
            % Restriction when Counter Reset is enabled
            if strcmpi(obj.Gpio2Sel, 'reset_counter')
                if (logical(obj.Atti) || logical(obj.Quaternion))
                    fprintf('Error: Atti or Quaternion cannot be enabled ');
                    fprintf('when reset_counter is enabled.');
                    fprintf('Disabling Atti, or Quaternion as fallback.');
                    obj.Atti = 0;
                    obj.Quaternion = 0;
                end
            end
            obj.setGpio2();
            % Restriction Atti and Quaternion cannot be both enabled
            if bitand(logical(obj.Atti), logical(obj.Quaternion))
                fprintf('Error: Atti & Quaternion cannot be both enabled.\n');
                fprintf('Disabling both as fallback.\n');
                obj.Atti = 0;
                obj.Quaternion = 0;
            end
            obj.setBrstCtrl();
            obj.setSigCtrl();
            obj.setAttiCtrl();
            obj.setUartCtrl();
            if logical(obj.is16G)
               obj.writeRegH(obj.GLOB_CMD3, hex2dec('1'));  % output range A_RANGE_CTRL
               obj.SF_ACC = 1/2;
               obj.SF_DLTV = 1.004e-2;

            end
        end

        function testSelf(obj)
        % Sets Selftest bit and waits for bit to clear
        % and checks result in DIAG_STAT
            if obj.Sampling == 1
                obj.gotoConfig();
            end
            fprintf('Selftest');
            selftest_bit = 4;
            obj.writeRegH(obj.MSC_CTRL, selftest_bit);
            tmprd = selftest_bit;
            while (tmprd == selftest_bit)
                tmp = obj.readReg(obj.MSC_CTRL);
                tmprd = tmp(1);
                fprintf('.');
                pause(obj.SELFTEST_DELAY);
            end
            result = obj.readReg(obj.DIAG_STAT);
            if bitand(result(2), 2) == 0
                fprintf('Pass\n');
            else
                fprintf('ST_ERR\n');
            end
        end

        function testFlash(obj)
        % Sets Flash test bit and waits for bit to clear
        % and checks result in DIAG_STAT
            if obj.Sampling == 1
                obj.gotoConfig();
            end
            fprintf('Flash');
            flash_bit = 8;
            obj.writeRegH(obj.MSC_CTRL, flash_bit);
            tmprd = flash_bit;
            while (tmprd == flash_bit)
                tmp = obj.readReg(obj.MSC_CTRL);
                tmprd = tmp(1);
                fprintf('.');
                pause(obj.FLASHTEST_DELAY);
            end
            result = obj.readReg(obj.DIAG_STAT);
            if bitand(result(2), 4) == 0
                fprintf('Pass\n');
            else
                fprintf('FLASH_ERR\n');
            end
        end

        function scaledBurstData = getScaledSamples(obj, nsamples)
        % Call getSampling() and convert data to scaled values
            if(nargin > 0)
                nrows = nsamples;
            else
                nrows = 1;
            end
            raw = obj.getSamples(nrows);
            scaled = zeros(nrows, obj.FieldsPerBurst);

            for i = 1: nrows
                j = 1;
                k = 1;
                if obj.Ndflag
                    scaled(i, j) = uint16(swapbytes(typecast(uint8(raw(i, k: k+1)),'uint16')));
                    j = j+1;
                    k = k+2;
                end
                if obj.TempC == 2
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_TEMPC/65536 + 25;
                    j = j+1;
                    k = k+4;
                elseif obj.TempC == 1
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_TEMPC + 25;
                    j = j+1;
                    k = k+2;
                end
                if obj.Gyro == 2
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_GYR/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_GYR/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_GYR/65536;
                    j = j+1;
                    k = k+4;
                elseif obj.Gyro == 1
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_GYR;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_GYR;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_GYR;
                    j = j+1;
                    k = k+2;
                end
                if obj.Accl == 2
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ACC/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ACC/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ACC/65536;
                    j = j+1;
                    k = k+4;
                elseif obj.Accl == 1
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_ACC;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_ACC;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_ACC;
                    j = j+1;
                    k = k+2;
                end
                if obj.DeltaA == 2
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_DLTA/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_DLTA/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_DLTA/65536;
                    j = j+1;
                    k = k+4;
                elseif obj.DeltaA == 1
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_DLTA;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_DLTA;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_DLTA;
                    j = j+1;
                    k = k+2;
                end
                if obj.DeltaV == 2
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_DLTV/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_DLTV/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_DLTV/65536;
                    j = j+1;
                    k = k+4;
                elseif obj.DeltaV == 1
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_DLTV;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_DLTV;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_DLTV;
                    j = j+1;
                    k = k+2;
                end
                if obj.Quaternion == 2
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) /2^30;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) /2^30;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) /2^30;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) /2^30;
                    j = j+1;
                    k = k+4;
                elseif obj.Quaternion == 1
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) /2^14;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) /2^14;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) /2^14;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) /2^14;
                    j = j+1;
                    k = k+2;
                end
                if obj.Atti == 2
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ANG/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ANG/65536;
                    j = j+1;
                    k = k+4;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ANG/65536;
                    j = j+1;
                    k = k+4;
                elseif obj.Atti == 1
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_ANG;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_ANG;
                    j = j+1;
                    k = k+2;
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+1)),'int16'))) * obj.SF_ANG;
                    j = j+1;
                    k = k+2;
                end
                if obj.Gpio
                    scaled(i, j) = uint16(swapbytes(typecast(uint8(raw(i, k: k+1)),'uint16')));
                    j = j+1;
                    k = k+2;
                end
                if obj.Counter
                    scaled(i, j) = uint16(swapbytes(typecast(uint8(raw(i, k: k+1)),'uint16')));
                    j = j+1;
                    k = k+2;
                end
                if obj.Chksm16
                    scaled(i, j) = uint16(swapbytes(typecast(uint8(raw(i, k: k+1)),'uint16')));
                    % calculate chksum16 on host side
                    calcChksm16 = bitand(sum(swapbytes(typecast(uint8(raw(i,1:end-2)),'uint16'))), 65535);
                    % compare chksum16
                    if scaled(i,j) ~= calcChksm16
                        fprintf('Checksum Mismatch\n');
                    end
                end
            end
            scaledBurstData.fields = obj.FieldsInBurst;
            scaledBurstData.samples = scaled;
        end

        function dumpReg(obj)
        % Read back all register values
            obj.readReg(obj.MODE_CTRL, true);
            obj.readReg(obj.DIAG_STAT, true);
            obj.readReg(obj.FLAG, true);
            obj.readReg(obj.GPIO, true);
            obj.readReg(obj.COUNT, true);
            obj.readReg(obj.RANGE_OVER, true);
            obj.readReg(obj.TEMP_HIGH, true);
            obj.readReg(obj.TEMP_LOW, true);
            obj.readReg(obj.XGYRO_HIGH, true);
            obj.readReg(obj.XGYRO_LOW, true);
            obj.readReg(obj.YGYRO_HIGH, true);
            obj.readReg(obj.YGYRO_LOW, true);
            obj.readReg(obj.ZGYRO_HIGH, true);
            obj.readReg(obj.ZGYRO_LOW, true);
            obj.readReg(obj.XACCL_HIGH, true);
            obj.readReg(obj.XACCL_LOW, true);
            obj.readReg(obj.YACCL_HIGH, true);
            obj.readReg(obj.YACCL_LOW, true);
            obj.readReg(obj.ZACCL_HIGH, true);
            obj.readReg(obj.ZACCL_LOW, true);
            obj.readReg(obj.ID, true);

            obj.readReg(obj.QTN0_HIGH, true);
            obj.readReg(obj.QTN0_LOW, true);
            obj.readReg(obj.QTN1_HIGH, true);
            obj.readReg(obj.QTN1_LOW, true);
            obj.readReg(obj.QTN2_HIGH, true);
            obj.readReg(obj.QTN2_LOW, true);
            obj.readReg(obj.QTN3_HIGH, true);
            obj.readReg(obj.QTN3_LOW, true);
            obj.readReg(obj.XDLTA_HIGH, true);
            obj.readReg(obj.XDLTA_LOW, true);
            obj.readReg(obj.YDLTA_HIGH, true);
            obj.readReg(obj.YDLTA_LOW, true);
            obj.readReg(obj.ZDLTA_HIGH, true);
            obj.readReg(obj.ZDLTA_LOW, true);
            obj.readReg(obj.XDLTV_HIGH, true);
            obj.readReg(obj.XDLTV_LOW, true);
            obj.readReg(obj.YDLTV_HIGH, true);
            obj.readReg(obj.YDLTV_LOW, true);
            obj.readReg(obj.ZDLTV_HIGH, true);
            obj.readReg(obj.ZDLTV_LOW, true);

            obj.readReg(obj.SIG_CTRL, true);
            obj.readReg(obj.MSC_CTRL, true);
            obj.readReg(obj.SMPL_CTRL, true);
            obj.readReg(obj.FILTER_CTRL, true);
            obj.readReg(obj.UART_CTRL, true);
            obj.readReg(obj.GLOB_CMD, true);
            obj.readReg(obj.BURST_CTRL1, true);
            obj.readReg(obj.BURST_CTRL2, true);
            obj.readReg(obj.POL_CTRL, true);
            obj.readReg(obj.GLOB_CMD3, true);
            obj.readReg(obj.ATTI_CTRL, true);
            obj.readReg(obj.GLOB_CMD2, true);
            obj.readReg(obj.R_MATRIX_M11, true);
            obj.readReg(obj.R_MATRIX_M12, true);
            obj.readReg(obj.R_MATRIX_M13, true);
            obj.readReg(obj.R_MATRIX_M21, true);
            obj.readReg(obj.R_MATRIX_M22, true);
            obj.readReg(obj.R_MATRIX_M23, true);
            obj.readReg(obj.R_MATRIX_M31, true);
            obj.readReg(obj.R_MATRIX_M32, true);
            obj.readReg(obj.R_MATRIX_M33, true);
            obj.readReg(obj.PROD_ID1, true);
            obj.readReg(obj.PROD_ID2, true);
            obj.readReg(obj.PROD_ID3, true);
            obj.readReg(obj.PROD_ID4, true);
            obj.readReg(obj.VERSION, true);
            obj.readReg(obj.SER_NUM1, true);
            obj.readReg(obj.SER_NUM2, true);
            obj.readReg(obj.SER_NUM3, true);
            obj.readReg(obj.SER_NUM4, true);
        end

        function printTabular(~, scale_data)
            % Print Scaled Data in Tabular output
            [~, col] = size(scale_data.fields);
            fprintf('Sample\t');
            for i = 1:col
                fprintf('%s\t', scale_data.fields(i));
            end
            fprintf('\n');

            [row, col] = size(scale_data.samples);
            for i = 1: row
                fprintf('%i\t', i);
                for j = 1: col
                    if startsWith(scale_data.fields(j), "Nd")
                        fprintf('%x\t', scale_data.samples(i, j));
                    elseif startsWith(scale_data.fields(j), "Cou")
                        fprintf('%i\t', scale_data.samples(i, j));
                    elseif startsWith(scale_data.fields(j), "Chk")
                        fprintf('%x\t', scale_data.samples(i, j));
                    elseif startsWith(scale_data.fields(j), "Gpi")
                        fprintf('%x\t', scale_data.samples(i, j));
                    elseif startsWith(scale_data.fields(j), "Gyr")
                        fprintf('%4.6f\t', scale_data.samples(i, j));
                    elseif startsWith(scale_data.fields(j), "Acc")
                        fprintf('%4.6f\t', scale_data.samples(i, j));
                    elseif startsWith(scale_data.fields(j), "Dlt")
                        fprintf('%4.6f\t', scale_data.samples(i, j));
                    elseif startsWith(scale_data.fields(j), "Qtn")
                        fprintf('%1.9f\t', scale_data.samples(i, j));
                    elseif startsWith(scale_data.fields(j), "Ang")
                        fprintf('%3.7f\t', scale_data.samples(i, j));
                    else
                        % Should never reach here
                        fprintf('%4.6f\t', scale_data.samples(i, j));
                    end
                end
                fprintf('\n');
            end
        end
    end


    methods(Access = 'private')
    % These methods are private, (i.e. only accessible from methods of
    % this class).

        function decodeExtSel(obj, mscctrl_lo)
        % Decode MSC_CTRL register value to Gpio2Sel
            switch (mscctrl_lo)
                case hex2dec('00')
                    obj.Gpio2Sel = 'gpio';
                case hex2dec('40')
                    obj.Gpio2Sel = 'reset_counter';
                case hex2dec('C0')
                    obj.Gpio2Sel = 'external_trigger';
                otherwise
                    fprintf('Invalid setting. No Change');
            end
        end

        function decodeDoutRate(obj, doutRate)
        % Decode DOUT_RATE register value to output rate in Hz
            switch (doutRate)
                case 0
                    obj.DoutRate = 2000;
                case 1
                    obj.DoutRate = 1000;
                case 2
                    obj.DoutRate = 500;
                case 3
                    obj.DoutRate = 250;
                case 4
                    obj.DoutRate = 125;
                case 5
                    obj.DoutRate = 62.5;
                case 6
                    obj.DoutRate = 31.25;
                case 7
                    obj.DoutRate = 15.625;
                case 8
                    obj.DoutRate = 400;
                case 9
                    obj.DoutRate = 200;
                case 10
                    obj.DoutRate = 100;
                case 11
                    obj.DoutRate = 80;
                case 12
                    obj.DoutRate = 50;
                case 13
                    obj.DoutRate = 40;
                case 14
                    obj.DoutRate = 25;
                case 15
                    obj.DoutRate = 20;
                otherwise
                    fprintf('Invalid Output Rate. No Change');
            end
        end

        function decodeFilterSel(obj, filterSel)
        % Decode FILTER_SEL register value to filter setting as string
            switch (filterSel)
                case 0
                    obj.FilterSel = 'tap0';
                case 1
                    obj.FilterSel = 'tap2';
                case 2
                    obj.FilterSel = 'tap4';
                case 3
                    obj.FilterSel = 'tap8';
                case 4
                    obj.FilterSel = 'tap16';
                case 5
                    obj.FilterSel = 'tap32';
                case 6
                    obj.FilterSel = 'tap64';
                case 7
                    obj.FilterSel = 'tap128';
                case 8
                    obj.FilterSel = '32fc50';
                case 9
                    obj.FilterSel = '32fc100';
                case 10
                    obj.FilterSel = '32fc200';
                case 11
                    obj.FilterSel = '32fc400';
                case 12
                    obj.FilterSel = '64fc50';
                case 13
                    obj.FilterSel = '64fc100';
                case 14
                    obj.FilterSel = '64fc200';
                case 15
                    obj.FilterSel = '64fc400';
                case 16
                    obj.FilterSel = '128fc50';
                case 17
                    obj.FilterSel = '128fc100';
                case 18
                    obj.FilterSel = '128fc200';
                case 19
                    obj.FilterSel = '128fc400';
                otherwise
                    fprintf('Invalid Filter. No Change');
            end
        end

        function setOutputRate(obj)
        % Set DOUT_RATE register from DoutRate property
            switch(obj.DoutRate)
                case 2000
                    wbyte = 0;
                case 1000
                    wbyte = 1;
                case 500
                    wbyte = 2;
                case 250
                    wbyte = 3;
                case 125
                    wbyte = 4;
                case 62.5
                    wbyte = 5;
                case 31.25
                    wbyte = 6;
                case 15.625
                    wbyte = 7;
                case 400
                    wbyte = 8;
                case 200
                    wbyte = 9;
                case 100
                    wbyte = 10;
                case 80
                    wbyte = 11;
                case 50
                    wbyte = 12;
                case 40
                    wbyte = 13;
                case 25
                    wbyte = 14;
                case 20
                    wbyte = 15;
                otherwise
                    fprintf('Invalid Output Rate. \n');
                    fprintf('Defaulting to 250\n');
                    wbyte = 3;
                    obj.DoutRate = 250;
            end
            obj.writeRegH(obj.SMPL_CTRL, wbyte);
        end

        function setFilter(obj)
        % Set FILTER_SEL register from FilterSel property
            switch(obj.FilterSel)
                case 'tap0'
                    wbyte = hex2dec('0');
                case 'tap2'
                    wbyte = hex2dec('1');
                case 'tap4'
                    wbyte = hex2dec('2');
                case 'tap8'
                    wbyte = hex2dec('3');
                case 'tap16'
                    wbyte = hex2dec('4');
                case 'tap32'
                    wbyte = hex2dec('5');
                case 'tap64'
                    wbyte = hex2dec('6');
                case 'tap128'
                    wbyte = hex2dec('7');
                case '32fc50'
                    wbyte = hex2dec('8');
                case '32fc100'
                    wbyte = hex2dec('9');
                case '32fc200'
                    wbyte = hex2dec('A');
                case '32fc400'
                    wbyte = hex2dec('B');
                case '64fc50'
                    wbyte = hex2dec('C');
                case '64fc100'
                    wbyte = hex2dec('D');
                case '64fc200'
                    wbyte = hex2dec('E');
                case '64fc400'
                    wbyte = hex2dec('F');
                case '128fc50'
                    wbyte = hex2dec('10');
                case '128fc100'
                    wbyte = hex2dec('11');
                case '128fc200'
                    wbyte = hex2dec('12');
                case '128fc400'
                    wbyte = hex2dec('13');
                otherwise
                    fprintf('Invalid Filter.\n');
                    fprintf('Must be tap0, tap2, tap4,\n');
                    fprintf('tap8, tap16, tap32, tap64, tap128,\n');
                    fprintf('32fc50, 32fc100, 32fc200, 32fc400\n');
                    fprintf('64fc50, 64fc100, 64fc200, 64fc400,\n');
                    fprintf('128fc50, 128fc100, 128fc200, 128fc400\n');
                    fprintf('Defaulting to tap16\n');
                    wbyte = hex2dec('4');
                    obj.FilterSel = 'tap16';
            end
            obj.writeRegL(obj.FILTER_CTRL, wbyte);
        end

        function setGpio2(obj)
        % Set GPIO2 based on Gpio2Sel property
            tmp = obj.readReg(obj.MSC_CTRL);
            tmp = bitand(tmp(2), hex2dec('3F'));
            if strcmpi(obj.Gpio2Sel, 'gpio')
                wbyte = tmp;
            elseif strcmpi(obj.Gpio2Sel, 'counter_reset')
                wbyte = bitor(tmp, hex2dec('40'));
            elseif strcmpi(obj.Gpio2Sel, 'external_trigger')
                wbyte = bitor(tmp, hex2dec('C0'));
            else
                fprintf('Invalid selection\n');
                fprintf('Defaulting to gpio\n');
                wbyte = tmp;
            end
            obj.writeRegL(obj.MSC_CTRL, wbyte);
        end

        function updateFieldsInBurst(obj)
        % Update FieldsInBurst based on object properties
            tmp = [];
            if obj.Ndflag
                tmp = "Ndflag";
            end
            if obj.TempC == 2
                tmp = [tmp "Temp32C"];
            elseif obj.TempC == 1
                tmp = [tmp "Temp16C"];
            end
            if obj.Gyro == 2
                tmp = [tmp "Gyro32X" "Gyro32Y" "Gyro32Z"];
            elseif obj.Gyro == 1
                tmp = [tmp "Gyro16X" "Gyro16Y" "Gyro16Z"];
            end
            if obj.Accl == 2
                tmp = [tmp "Accl32X" "Accl32Y" "Accl32Z"];
            elseif obj.Accl == 1
                tmp = [tmp "Accl16X" "Accl16Y" "Accl16Z"];
            end
            if obj.DeltaA == 2
                tmp = [tmp "DltA32X" "DltA32Y" "DltA32Z"];
            elseif obj.DeltaA == 1
                tmp = [tmp "DltA16X" "DltA16Y" "DltA16Z"];
            end
            if obj.DeltaV == 2
                tmp = [tmp "DltV32X" "DltV32Y" "DltV32Z"];
            elseif obj.DeltaV == 1
                tmp = [tmp "DltV16X" "DltV16Y" "DltV16Z"];
            end
            if obj.Quaternion == 2
                tmp = [tmp "Qtn32-0" "Qtn32-1" "Qtn32-2" "Qtn32-3"];
            elseif obj.Quaternion == 1
                tmp = [tmp "Qtn16-0" "Qtn16-1" "Qtn16-2" "Qtn16-3"];
            end
            if obj.Atti == 2
                tmp = [tmp "Ang32-1" "Ang32-2" "Ang32-3"];
            elseif obj.Atti == 1
                tmp = [tmp "Ang16-1" "Ang16-2" "Ang16-3"];
            end
            if obj.Gpio
                tmp = [tmp "Gpio"];
            end
            if obj.Counter
                tmp = [tmp "Count"];
            end
            if obj.Chksm16
                tmp = [tmp "Chksm16"];
            end
            obj.FieldsInBurst = tmp;
        end

        function updateBytesPerBurst(obj)
        % Update BytesPerBurst based on Properties
            tmp = logical(obj.Ndflag) * 2 + ...
                obj.TempC * 2 + ...
                obj.Gyro * 3 * 2 + ...
                obj.Accl * 3 * 2 + ...
                obj.DeltaA * 3 * 2 + ...
                obj.DeltaV * 3 * 2 + ...
                obj.Quaternion * 4 * 2 + ...
                obj.Atti * 3 * 2 + ...
                logical(obj.Gpio) * 2 + ...
                logical(obj.Counter) * 2 + ...
                logical(obj.Chksm16) * 2 + ...
                2; % Header Byte, Delimiter Byte
            obj.BytesPerBurst = tmp;
        end

        function updateFieldsPerBurst(obj)
        % Update FieldsPerBurst based on Properties
            tmp = logical(obj.Ndflag) + ...
                logical(obj.TempC) + ...
                logical(obj.Gyro) * 3 + ...
                logical(obj.Accl) * 3 + ...
                logical(obj.DeltaA) * 3 + ...
                logical(obj.DeltaV) * 3 + ...
                logical(obj.Quaternion) * 4 + ...
                logical(obj.Atti) * 3 + ...
                logical(obj.Gpio) + ...
                logical(obj.Counter) + ...
                logical(obj.Chksm16);
            obj.FieldsPerBurst = tmp;
        end

        function setBrstCtrl(obj)
        % Set BURST_CTRL1 & BURST_CTRL2 based on object properties
        % Update BytesPerBurst & FieldsInBurst properties
            tmp_hi = logical(obj.Ndflag) * 2^7 + ...
                logical(obj.TempC) * 2^6 + ...
                logical(obj.Gyro) * 2^5 + ...
                logical(obj.Accl) * 2^4 + ...
                logical(obj.DeltaA) * 2^3 + ...
                logical(obj.DeltaV) * 2^2 + ...
                logical(obj.Quaternion) * 2^1 + ...
                logical(obj.Atti);
            tmp_lo = logical(obj.Gpio) * 2^2 + ...
                logical(obj.Counter) * 2^1 + ...
                logical(obj.Chksm16);
            % Write to BURST_CTRL1
            obj.writeRegH(obj.BURST_CTRL1, tmp_hi);
            obj.writeRegL(obj.BURST_CTRL1, tmp_lo);
            % BURST_CTRL2
            tmp_hi = 0;
            if obj.TempC == 2
                tmp_hi = tmp_hi + 2^6;
            end
            if obj.Gyro == 2
                tmp_hi = tmp_hi + 2^5;
            end
            if obj.Accl == 2
                tmp_hi = tmp_hi + 2^4;
            end
            if obj.DeltaA == 2
                tmp_hi = tmp_hi + 2^3;
            end
            if obj.DeltaV == 2
                tmp_hi = tmp_hi + 2^2;
            end
            if obj.Quaternion == 2
                tmp_hi = tmp_hi + 2^1;
            end
            if obj.Atti == 2
                tmp_hi = tmp_hi + 1;
            end
            % Write to BURST_CTRL2
            obj.writeRegH(obj.BURST_CTRL2, tmp_hi);
            % Update BytesPerBurst property
            obj.updateBytesPerBurst();
            % Update FieldsPerBurst property
            obj.updateFieldsPerBurst();
            % Update FieldsInBurst property
            obj.updateFieldsInBurst();
        end

        function setSigCtrl(obj)
        % Set SIG_CTRL based on object properties
            % Create Write Byte based on properties
            tmp_hi = logical(obj.TempC) * 2^7 + ...
                logical(obj.Gyro) * 7 * 2^4 + ...
                logical(obj.Accl) * 7 * 2^1;
            tmp_lo = logical(obj.DeltaA) * 7 * 2^5 + ...
                logical(obj.DeltaV) * 7 * 2^2;

            % Write to SIG_CTRL
            obj.writeRegH(obj.SIG_CTRL, tmp_hi);
            obj.writeRegL(obj.SIG_CTRL, tmp_lo);
        end

        function setAttiCtrl(obj)
        % Set ATTI_CTRL based on object properties
            if (bitor(logical(obj.Atti), logical(obj.Quaternion)))
                obj.writeRegH(obj.ATTI_CTRL, hex2dec('0C'));
            elseif bitor(logical(obj.DeltaA), logical(obj.DeltaV))
                obj.writeRegH(obj.ATTI_CTRL, hex2dec('02'));
            else
                obj.writeRegH(obj.ATTI_CTRL, hex2dec('00'));
            end
        end

        function setUartCtrl(obj)
        % Set UART_CTRL based on object properties
            if (logical(obj.UartAuto))
                obj.writeRegL(obj.UART_CTRL, hex2dec('01'));
            else
                obj.writeRegL(obj.UART_CTRL, hex2dec('00'));
            end
        end


    end

end