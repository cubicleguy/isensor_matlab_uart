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
% Epson A352 Accelerometer derived from Epson Device Uart handle class

classdef A352Accl < EpsonDeviceUart

    properties
        TiltX (1,1) {mustBeInteger, ... % Enable Tilt for X-axis in burst sample instead of acceleration
            mustBeNonnegative, mustBeLessThan(TiltX, 2)} = 0;
        TiltY (1,1) {mustBeInteger, ... % Enable Tilt for Y-axis in burst sample instead of acceleration
            mustBeNonnegative, mustBeLessThan(TiltY, 2)} = 0;
        TiltZ (1,1) {mustBeInteger, ... % Enable Tilt for Z-axis in burst sample instead of acceleration
            mustBeNonnegative, mustBeLessThan(TiltZ, 2)} = 0;
        DoutRate (1,1) {mustBeInteger, mustBeNonnegative, ... % Output rate in Hz
            mustBeLessThan(DoutRate, 1001)} = 200;
        FilterSel (1,:) char {mustBeMember(FilterSel,{... % Filter selection
            '512fc460', '512fc210', '512fc60', '512fc16', '512fc9',...
            '128fc350', '128fc110', '128fc36',...
            '64fc220', '64fc83'})} = '512fc60';
        ReducedNoiseEn (1,1) {mustBeInteger, ... % Enable Reduce Noise Floor mode
            mustBeNonnegative, mustBeLessThan(ReducedNoiseEn, 2)} = 0;
        TempStabEn (1,1) {mustBeInteger, ... % Enable Bias Stabilization Against Temperature mode
            mustBeNonnegative, mustBeLessThan(TempStabEn, 2)} = 0;
        ExtEnable (1,1) {mustBeInteger, ... % Enable External Trigger
            mustBeNonnegative, mustBeLessThan(ExtEnable, 2)} = 0;
    end

    properties(GetAccess = 'public', SetAccess = 'private', Hidden=true)
        % Scalefactors
        SF_ACC = 1000/2^24; % Scale factor to convert int32 to mG (milli-G)
        SF_TILT = 1/2^29; % Scale factor to convert int32 to radians
        SF_TEMPC = -0.0037918/2^16; % Scale factor to convert int32 to degC
        TEMPC_OFFSET = 34.987; % Offset for conversion to degC
    end

    properties(Constant, Hidden=true)
        % Register Addresses
        TEMP_HIGH = [0, hex2dec('0E')];
        TEMP_LOW = [0, hex2dec('10')];
        XACCL_HIGH = [0, hex2dec('30')];
        XACCL_LOW = [0, hex2dec('32')];
        YACCL_HIGH = [0, hex2dec('34')];
        YACCL_LOW = [0, hex2dec('36')];
        ZACCL_HIGH = [0, hex2dec('38')];
        ZACCL_LOW = [0, hex2dec('3A')];
        XTILT_HIGH = [0, hex2dec('3C')];
        XTILT_LOW = [0, hex2dec('3E')];
        YTILT_HIGH = [0, hex2dec('40')];
        YTILT_LOW = [0, hex2dec('42')];
        ZTILT_HIGH = [0, hex2dec('44')];
        ZTILT_LOW = [0, hex2dec('46')];
        BURST_CTRL = [1, hex2dec('0C')];
        FIR_UCMD = [1, hex2dec('16')];
        FIR_UDATA = [1, hex2dec('18')];
        FIR_UADDR = [1, hex2dec('1A')];
        LONGFILT_CTRL = [1, hex2dec('1C')];
        LONGFILT_TAP = [1, hex2dec('1E')];
        OFFSET_XA_HIGH = [1, hex2dec('2C')];
        OFFSET_XA_LOW = [1, hex2dec('2E')];
        OFFSET_YA_HIGH = [1, hex2dec('30')];
        OFFSET_YA_LOW = [1, hex2dec('32')];
        OFFSET_ZA_HIGH = [1, hex2dec('34')];
        OFFSET_ZA_LOW = [1, hex2dec('36')];
        XALARM = [1, hex2dec('46')];
        YALARM = [1, hex2dec('48')];
        ZALARM = [1, hex2dec('4A')];
        % Device Timings
        SELFTEST_DELAY = 200e-3;
        SNSVTYTEST_DELAY = 40;
        FLASHTEST_DELAY = 5e-3;
        FLASHBACKUP_DELAY = 310e-3;
        FLASHRESET_DELAY = 1900e-3;
        FILTER_DELAY = 4e-3;
        UDFFILTER_DELAY = 100e-3;
    end

    methods

        function obj = A352Accl(comport, baudrate)
        % class constructor
            if not(exist('comport', 'var'))
                error('Error: Must specify comport');
            end
            if not(exist('baudrate', 'var'))
                error('Error: Must specify baudrate');
            end
            obj@EpsonDeviceUart(comport, baudrate);
            obj.getDeviceCfg();
            if not(strcmpi(obj.ProdId(1:4), 'A352'))
                fprintf('\n***Detected mismatch of Product ID and object***\n');
                fprintf('***Check the device matches the selected class object***\n');
            end
        end

        function getDeviceCfg(obj)
        % Update object properties by reading device registers
        % Ndflag, TempC, Counter, Chksm16
        % TiltX, TiltY, TiltZ, DoutRate, FilterSel
        % ReducedNoiseEn, TempStabEn, ExtEnable
            % In UART_AUTO & SAMPLING need to goto CONFIG mode and clear
            % input buffer
            if (obj.Sampling && obj.UartAuto)
                obj.gotoConfig();
            end

            sigCtrl = obj.readReg(obj.SIG_CTRL);
            obj.TiltX = any(bitand(sigCtrl(2), hex2dec('80')));
            obj.TiltY = any(bitand(sigCtrl(2), hex2dec('40')));
            obj.TiltZ = any(bitand(sigCtrl(2), hex2dec('20')));
            obj.ReducedNoiseEn = any(bitand(sigCtrl(2), hex2dec('10')));
            obj.TempStabEn = any(bitand(sigCtrl(2), hex2dec('04')));
            % Read EXT function setting
            mscCtrl = obj.readReg(obj.MSC_CTRL);
            obj.ExtEnable = any(bitand(mscCtrl(2), hex2dec('40')));

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
            burstCtrl = obj.readReg(obj.BURST_CTRL);
            % NDFLAG
            obj.Ndflag = any(bitand(burstCtrl(1), hex2dec('80')));
            % TEMPC
            obj.TempC = any(bitand(burstCtrl(1), hex2dec('40')));
            % Counter
            obj.Counter = any(bitand(burstCtrl(2), hex2dec('02')));
            % Chksm16
            obj.Chksm16 = any(bitand(burstCtrl(2), hex2dec('01')));
            obj.updateBytesPerBurst();
            obj.updateFieldsPerBurst();
            obj.updateFieldsInBurst();
        end

        function setDeviceCfg(obj)
        % If not already goto CONFIG mode and program device registers
        % SMPL_CTRL, FILTER_CTRL, MSC_CTRL, BURST_CTRL,
        % SIG_CTRL, UART_CTRL based on object properties
        % Will automatically enter CONFIG mode
            % Register writes need to be in CONFIG mode and flush
            % any data in input buffer
            if obj.Sampling == 1
                obj.gotoConfig();
            end
            obj.setOutputRate();
            obj.setFilter();
            obj.setExt();
            obj.setBrstCtrl();
            obj.setSigCtrl();
            obj.setUartCtrl();
        end

        function doSelftests(obj)
        % Perform device self tests
            if obj.Sampling == 1
                obj.gotoConfig();
            end
            obj.testSENS(1*2^6);
            obj.testSENS(1*2^5);
            obj.testSENS(1*2^4);
            obj.testFlash();
            obj.testACC();
            obj.testTempC();
            obj.testVDD();
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
                if obj.TempC
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_TEMPC + obj.TEMPC_OFFSET;
                    j = j+1;
                    k = k+4;
                end
                if obj.TiltX
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_TILT;
                    j = j+1;
                    k = k+4;
                else
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ACC;
                    j = j+1;
                    k = k+4;
                end
                if obj.TiltY
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_TILT;
                    j = j+1;
                    k = k+4;
                else
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ACC;
                    j = j+1;
                    k = k+4;
                end
                if obj.TiltZ
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_TILT;
                    j = j+1;
                    k = k+4;
                else
                    scaled(i, j) = double(swapbytes(typecast(uint8(raw(i, k: k+3)),'int32'))) * obj.SF_ACC;
                    j = j+1;
                    k = k+4;
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
            obj.readReg(obj.COUNT, true);
            obj.readReg(obj.TEMP_HIGH, true);
            obj.readReg(obj.TEMP_LOW, true);
            obj.readReg(obj.XACCL_HIGH, true);
            obj.readReg(obj.XACCL_LOW, true);
            obj.readReg(obj.YACCL_HIGH, true);
            obj.readReg(obj.YACCL_LOW, true);
            obj.readReg(obj.ZACCL_HIGH, true);
            obj.readReg(obj.ZACCL_LOW, true);
            obj.readReg(obj.XTILT_HIGH, true);
            obj.readReg(obj.XTILT_LOW, true);
            obj.readReg(obj.YTILT_HIGH, true);
            obj.readReg(obj.YTILT_LOW, true);
            obj.readReg(obj.ZTILT_HIGH, true);
            obj.readReg(obj.ZTILT_LOW, true);
            obj.readReg(obj.ID, true);
            obj.readReg(obj.SIG_CTRL, true);
            obj.readReg(obj.MSC_CTRL, true);
            obj.readReg(obj.SMPL_CTRL, true);
            obj.readReg(obj.FILTER_CTRL, true);
            obj.readReg(obj.UART_CTRL, true);
            obj.readReg(obj.GLOB_CMD, true);
            obj.readReg(obj.BURST_CTRL, true);
            obj.readReg(obj.FIR_UCMD, true);
            obj.readReg(obj.FIR_UDATA, true);
            obj.readReg(obj.FIR_UADDR, true);
            obj.readReg(obj.LONGFILT_CTRL, true);
            obj.readReg(obj.LONGFILT_TAP, true);
            obj.readReg(obj.OFFSET_XA_HIGH, true);
            obj.readReg(obj.OFFSET_XA_LOW, true);
            obj.readReg(obj.OFFSET_YA_HIGH, true);
            obj.readReg(obj.OFFSET_YA_LOW, true);
            obj.readReg(obj.OFFSET_ZA_HIGH, true);
            obj.readReg(obj.OFFSET_ZA_LOW, true);
            obj.readReg(obj.XALARM, true);
            obj.readReg(obj.YALARM, true);
            obj.readReg(obj.ZALARM, true);
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

        function decodeDoutRate(obj, doutRate)
        % Decode DOUT_RATE register value to output rate in Hz
            switch (doutRate)
                case 2
                    obj.DoutRate = 1000;
                case 3
                    obj.DoutRate = 500;
                case 4
                    obj.DoutRate = 200;
                case 5
                    obj.DoutRate = 100;
                case 6
                    obj.DoutRate = 50;
                otherwise
                    fprintf('Invalid Output Rate. No Change');
            end
        end

        function decodeFilterSel(obj, filterSel)
        % Decode FILTER_SEL register value to filter setting as string
            switch (filterSel)
                case 10
                    obj.FilterSel = '512fc460';
                case 9
                    obj.FilterSel = '512fc210';
                case 8
                    obj.FilterSel = '512fc60';
                case 7
                    obj.FilterSel = '512fc16';
                case 6
                    obj.FilterSel = '512fc9';
                case 5
                    obj.FilterSel = '128fc350';
                case 4
                    obj.FilterSel = '128fc110';
                case 3
                    obj.FilterSel = '128fc36';
                case 2
                    obj.FilterSel = '64fc220';
                case 1
                    obj.FilterSel = '64fc220';
                otherwise
                    fprintf('Invalid Filter. No Change');
            end
        end

        function setOutputRate(obj)
        % Set DOUT_RATE register from DoutRate property
            if obj.DoutRate == 1000
                wbyte = 2;
            elseif obj.DoutRate == 500
                wbyte = 3;
            elseif obj.DoutRate == 200
                wbyte = 4;
            elseif obj.DoutRate == 100
                wbyte = 5;
            elseif obj.DoutRate == 50
                wbyte = 6;
            else
                fprintf('Invalid Output Rate. Must be 1000, 500, \n');
                fprintf('200, 100, or 50. Defaulting to 200\n');
                wbyte = 4;
                obj.DoutRate = 200;
            end
            obj.writeRegH(obj.SMPL_CTRL, wbyte);
        end

        function setFilter(obj)
        % Set FILTER_SEL register from FilterSel property
            if strcmpi(obj.FilterSel, '512fc460')
                wbyte = hex2dec('A');
            elseif strcmpi(obj.FilterSel, '512fc210')
                wbyte = hex2dec('9');
            elseif strcmpi(obj.FilterSel, '512fc60')
                wbyte = hex2dec('8');
            elseif strcmpi(obj.FilterSel, '512fc16')
                wbyte = hex2dec('7');
            elseif strcmpi(obj.FilterSel, '512fc9')
                wbyte = hex2dec('6');
            elseif strcmpi(obj.FilterSel, '128fc350')
                wbyte = hex2dec('5');
            elseif strcmpi(obj.FilterSel, '128fc110')
                wbyte = hex2dec('4');
            elseif strcmpi(obj.FilterSel, '128fc36')
                wbyte = hex2dec('3');
            elseif strcmpi(obj.FilterSel, '64fc220')
                wbyte = hex2dec('2');
            elseif strcmpi(obj.FilterSel, '64fc83')
                wbyte = hex2dec('1');
            else
                fprintf('Invalid Filter. Must be 512fc460, 512fc210, \n');
                fprintf('512fc60, 512fc16, 512fc9, 128fc350, 128fc110, \n');
                fprintf('128fc36, 64fc220, 64fc83. Defaulting to 512fc60\n');
                wbyte = hex2dec('8');
                obj.FilterSel = '512fc60';
            end
            obj.writeRegL(obj.FILTER_CTRL, wbyte);
        end

        function setExt(obj)
        % Set EXT Trigger based on ExtEnable property
            tmp = obj.readReg(obj.MSC_CTRL);
            tmp = tmp(2);
            if obj.ExtEnable == 1
                wbyte = bitor(tmp, hex2dec('40'));
            else
                wbyte = bitand(tmp, hex2dec('BF'));
            end
            obj.writeRegL(obj.MSC_CTRL, wbyte);
        end

        function updateFieldsInBurst(obj)
        % Update FieldsInBurst based on object properties
            tmp = ["AcclX", "AcclY", "AcclZ"];
            if obj.TiltX
                tmp(1, 1) = "TiltX";
            end
            if obj.TiltY
                tmp(1, 2) = "TiltY";
            end
            if obj.TiltZ
                tmp(1, 3) = "TiltZ";
            end
            if obj.TempC
                tmp = ["TempC", tmp];
            end
            if obj.Ndflag
                tmp = ["NdFlags", tmp];
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
                logical(obj.TempC) * 4 + ...
                (3 * 4) + ...
                logical(obj.Counter) * 2 + ...
                logical(obj.Chksm16) * 2 + ...
                2; % Header Byte, Delimiter Byte
            obj.BytesPerBurst = tmp;
        end

        function updateFieldsPerBurst(obj)
        % Update FieldsPerBurst based on Properties
            tmp = logical(obj.Ndflag) + ...
                logical(obj.TempC) + ...
                3 + ...
                logical(obj.Counter) + ...
                logical(obj.Chksm16);
            obj.FieldsPerBurst = tmp;
        end

        function setBrstCtrl(obj)
        % Set BURST_CTRL (Accl X, Y, Z always enabled) based on object properties
            % If Temperature is nonzero then it is a 1
            if (obj.TempC ~= 0)
                obj.TempC = 1;
            end
            tmp_hi = (obj.Ndflag * 2^7) + (obj.TempC * 2^6 + 7);
            tmp_lo = (obj.Counter * 2^1) + (obj.Chksm16);
            % Write to BURST_CTRL
            obj.writeRegH(obj.BURST_CTRL, tmp_hi);
            obj.writeRegL(obj.BURST_CTRL, tmp_lo);
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
            tmp_hi = logical(obj.TempC) * 2^7 + (7 * 2^1);
            tmp_lo = logical(obj.TiltX) * 2^7 + ...
                logical(obj.TiltY) * 2^6 + ...
                logical(obj.TiltZ) * 2^5 + ...
                logical(obj.ReducedNoiseEn) * 2^4 + ...
                logical(obj.TempStabEn) * 2^2;
            % Write to SIG_CTRL
            obj.writeRegH(obj.SIG_CTRL, tmp_hi);
            obj.writeRegL(obj.SIG_CTRL, tmp_lo);
        end

        function setUartCtrl(obj)
        % Set UART_CTRL based on object properties
            if (logical(obj.UartAuto))
                obj.writeRegL(obj.UART_CTRL, hex2dec('01'));
            else
                obj.writeRegL(obj.UART_CTRL, hex2dec('00'));
            end
        end

        function str_result = decodeSENS(~, result)
        % Decodes SENS selftest 2-bit result
            switch (result)
                case (hex2dec('08'))
                    str_result = 'Undetermined';
                case (hex2dec('04'))
                    str_result = 'Error';
                case (hex2dec('00'))
                    str_result = 'Pass';
                otherwise
                    str_result = 'Invalid Result';
            end
        end

        function testSENS(obj, testType)
        % Sets specific Self Test bit and waits for bit to clear
        % and decode result in DIAG_STAT
            switch (testType)
                case (1*2^6)
                    str_result = 'Z_SENS';
                case (1*2^5)
                    str_result = 'Y_SENS';
                case (1*2^4)
                    str_result = 'X_SENS';
                otherwise
                    str_result = 'Invalid';
            end
            fprintf('%s', str_result);
            obj.writeRegH(obj.MSC_CTRL, testType);
            tmprd = testType;
            while (tmprd == testType)
                tmp = obj.readReg(obj.MSC_CTRL);
                tmprd = tmp(1);
                fprintf('.');
                pause(1);
            end
            result = obj.readReg(obj.DIAG_STAT);
            %fprintf('%x %x ', result(1), result(2))
            fprintf('%s\n', obj.decodeSENS(result(1)));
        end

        function testFlash(obj)
        % Sets Flash test bit and waits for bit to clear
        % and checks result in DIAG_STAT
            fprintf('Flash');
            flash_bit = 8;
            obj.writeRegH(obj.MSC_CTRL, flash_bit);
            tmprd = flash_bit;
            while (tmprd == flash_bit)
                tmp = obj.readReg(obj.MSC_CTRL);
                tmprd = tmp(1);
                fprintf('.');
                pause(0.005);
            end
            result = obj.readReg(obj.DIAG_STAT);
            if bitand(result(2), 4) == 0
                fprintf('Pass\n');
            else
                fprintf('FLASH_ERR\n');
            end
        end

        function testACC(obj)
        % Sets ACC test bit and waits for bit to clear
        % and checks result in DIAG_STAT
            fprintf('ACC');
            ACC_bit = 4;
            obj.writeRegH(obj.MSC_CTRL, ACC_bit);
            tmprd = ACC_bit;
            while (tmprd == ACC_bit)
                tmp = obj.readReg(obj.MSC_CTRL);
                tmprd = tmp(1);
                fprintf('.');
                pause(0.005);
            end
            result = obj.readReg(obj.DIAG_STAT);
            if bitand(result(1), hex2dec('F0')) == 0
                fprintf('Pass\n');
            else
                fprintf('ACC_ERR\n');
            end
        end

        function testTempC(obj)
        % Sets TEMP test bit and waits for bit to clear
        % and checks result in DIAG_STAT
            fprintf('TEMP');
            TEMP_bit = 2;
            obj.writeRegH(obj.MSC_CTRL, TEMP_bit);
            tmprd = TEMP_bit;
            while (tmprd == TEMP_bit)
                tmp = obj.readReg(obj.MSC_CTRL);
                tmprd = tmp(1);
                fprintf('.');
                pause(0.005);
            end
            result = obj.readReg(obj.DIAG_STAT);
            if bitand(result(1), hex2dec('02')) == 0
                fprintf('Pass\n');
            else
                fprintf('TEMP_ERR\n');
            end
        end

        function testVDD(obj)
        % Sets VDD test bit and waits for bit to clear
        % and checks result in DIAG_STAT
            fprintf('VDD');
            VDD_bit = 1;
            obj.writeRegH(obj.MSC_CTRL, VDD_bit);
            tmprd = VDD_bit;
            while (tmprd == VDD_bit)
                tmp = obj.readReg(obj.MSC_CTRL);
                tmprd = tmp(1);
                fprintf('.');
                pause(0.005);
            end
            result = obj.readReg(obj.DIAG_STAT);
            if bitand(result(1), hex2dec('01')) == 0
                fprintf('Pass\n');
            else
                fprintf('VDD_ERR\n');
            end
        end

    end

end