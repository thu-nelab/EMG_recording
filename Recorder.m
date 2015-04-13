function Recorder
%% Predefines
frameFormatDict = struct('code', {'01'  '02'    '03'    '11'    '12'    '13'    '21'        'A1'},...
                       'format', {'EEG' 'EMG'   'ECG'   'GYRO'  'ACC'   'MAG'   'Impedance' 'Event'});
for i = 1:length(frameFormatDict), frameFormatDict(i).code = hex2dec(frameFormatDict(i).code); end;

sampleRateDict = struct('code', {'01'   '02'    '03'    '04'    '05'    '06'    '07'    '08'    '11'    '12'    '13'}, ...
                  'sampleRate', {250    500     1000    2000    4000    8000    16000   32000   10      50      100});
for i = 1:length(sampleRateDict), sampleRateDict(i).code = hex2dec(sampleRateDict(i).code); end;

cmdStart = {'10' '01' '04' '00' '01' '00' '00' '00' '00' '00' '00' '00' '00' '00' '00' '00' '00' '01' '01' '00'};
for i = 1:length(cmdStart), cmdStart{i} = hex2dec(cmdStart{i}); end; cmdStart = cell2mat(cmdStart);
cmdStop  = {'10' '01' '04' '00' '01' '00' '00' '00' '00' '00' '00' '00' '00' '00' '00' '00' '01' '01' '01' '00'};
for i = 1:length(cmdStop), cmdStop{i} = hex2dec(cmdStop{i}); end; cmdStop = cell2mat(cmdStop);

scaleEEG = 2^23 / 200000;
scaleEMG = 2^23 / 200000;
scaleGYRO = 57.1429;
scaleACC = 1;
scaleMAGX = 230; scaleMAGY = 230; scaleMAGZ = 205;
%% Serial Port
IOPort('CloseAll');
% serialPortName = '/dev/tty.Penguin002-BluetoothSer';
serialPortName = 'COM5';
[serialPort errmsg] = IOPort('OpenSerialPort', serialPortName, 'BaudRate=115200, HardwareBufferSizes=32768,8192');
IOPort('Purge', serialPort);
IOPort('Flush', serialPort);
%% Initialization
nPointMax = 1000*60*10;
nChanEMG = 8;
EEG = DataPackage(8, 100);
EMG = DataPackage(nChanEMG, nPointMax);
nChanGYRO = 3;
GYRO = DataPackage(nChanGYRO, nPointMax);
ACC = DataPackage(nChanGYRO, nPointMax);
MAG = DataPackage(nChanGYRO, nPointMax);
cmd=[];

assignin('base', 'EEG', EEG);
assignin('base', 'EMG', EMG);
assignin('base', 'GYRO', GYRO);
assignin('base', 'ACC', ACC);
assignin('base', 'MAG', MAG);

%% Start
IOPort('Write', serialPort, uint8(cmdStart),0);
%% Main Loop
raw = [];
tic;

while 1
     if IOPort('BytesAvailable', serialPort) > 0
        buffer = IOPort('Read', serialPort);
        raw = [raw buffer];
    end
    while length(raw) > 16
        if raw(1) == 240 && raw(2) == 15 % data package
            payloadLength = raw(3) + raw(4)*2^8;
            if length(raw) >= 16+payloadLength % complete data package received, 
                % parse data package
                frameFormat = frameFormatDict([frameFormatDict.code] == raw(5)).format;
                if isempty(frameFormat), fprintf('Invalid frameFormat at %d\n', 5); return; end;

                deviceID = raw(6);
                packetID = raw(7);

                sampleRate = sampleRateDict([sampleRateDict.code] == raw(8)).sampleRate;
                if isempty(sampleRate), fprintf('Invalid sampleRate at %d\n', 8); return; end;

                dataFormat = raw(9);

                nChan = raw(10);
                nPoint = raw(11) + raw(12)*2^8;
                
                timeStamp = double(typecast(uint8(raw(13 : 16)), 'uint32'));
                timeStamp = timeStamp : 1/sampleRate*1000 : timeStamp + 1/sampleRate*1000 * (nPoint-1);

                % parse raw data
                rawData = ParseRawData(raw(17 : 16+payloadLength), nChan, nPoint, dataFormat);
                
                % remove parsed package from raw
                raw(1:16+payloadLength) = [];
                                
                % append data
                switch frameFormat
                    case 'EEG'
                        sigData = rawData/scaleEEG;
                        EEG.AppendData(deviceID, sampleRate, rawData, timeStamp);
                    case 'EMG'
                        sigData = rawData/scaleEMG;
                        EMG.AppendData(deviceID, sampleRate, sigData, timeStamp);
                    case 'GYRO'
                        sigData = rawData/scaleGYRO;
                        GYRO.AppendData(deviceID, sampleRate, sigData, timeStamp);
%                         svGYRO.Update(sigData);
                    case 'ACC'
                        sigData = rawData/scaleACC;
                        ACC.AppendData(deviceID, sampleRate, sigData, timeStamp);
                    case 'MAG'
                        sigData = rawData/scaleMAGX;
                        MAG.AppendData(deviceID, sampleRate, sigData, timeStamp);
                end  
            else
                break;
            end
            
        elseif raw(1) == 32 && raw(2) == 2 % command package
            payloadLength = raw(3) + raw(4)*2^8;
            if length(raw) >= 16+payloadLength
                cmd = [cmd raw(17 : 16+payloadLength)];
                raw(1:16+payloadLength) = [];
            else
                break;
            end
            
        else
            fprintf('Invalid token at %d\n', i);
            return;
        end
    end
    pause(0.000001);
end
%% exit 
IOPort('CloseAll');
close all;
end
%%
function data = ParseRawData(raw, nChan, nPoint, dataFormat)
    endian = bitand(dataFormat, 128) / 2^7;
    type = bitand(dataFormat, 112) / 2^4;
    nByte = bitand(dataFormat, 15);

    if length(raw) ~= nByte*nChan*nPoint
        fprintf('Invalid data format\n');
        return;
    end

    data = zeros(nChan, nPoint);
    for iPoint = 1:nPoint
        for iChan = 1:nChan
            iCurrent = (iPoint-1)*nChan*nByte + (iChan-1)*nByte + 1;
            if endian == 1 % big endian
                byteArray = raw(iCurrent + nByte-1 : -1 : iCurrent);      
            else % little endian
                byteArray = raw(iCurrent : iCurrent + nByte-1);
            end

            switch type
                case 0 % uint
                    for iByte = 1:nByte
                        data(iChan, iPoint) = data(iChan, iPoint) + byteArray(iByte)*2^((iByte-1)*8);
                    end
                case 1 % int
                    signbit = bitand(byteArray(nByte), 128) / 2^7;
                    byteArray(nByte) = bitand(byteArray(nByte), 127);
                    for iByte = 1:nByte
                        data(iChan, iPoint) = data(iChan, iPoint) + byteArray(iByte)*2^((iByte-1)*8);
                    end
                    if signbit == 1
                        data(iChan, iPoint) = data(iChan, iPoint) - 2^(nByte*8 - 1);
                    end
            end
        end
    end
    data = double(data);
end
