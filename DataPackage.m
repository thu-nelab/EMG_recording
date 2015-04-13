classdef DataPackage < handle
    %DataPackage
    
    properties
        deviceID;
        sampleRate;
        nChan;
        nPointMax;
        nPoint;
        data;
        dataFlag;
        timeStamp;
    end
    
    methods
        
        function obj = DataPackage(nChan, nPointMax)
            obj.nChan = nChan;
            obj.nPointMax = nPointMax;
            obj.nPoint = 0;
            obj.data = zeros(nChan, nPointMax);
            obj.dataFlag = zeros(1, nPointMax);
            obj.timeStamp = zeros(1, nPointMax);
        end
        
        function AppendData(obj, deviceID, sampleRate, data, timeStamp)
            
            obj.deviceID = deviceID;
            obj.sampleRate = sampleRate;
            
            appendDataLength = size(data,2);
            
            obj.data(:,obj.nPoint+1:obj.nPoint+appendDataLength) = data;
            
            obj.timeStamp(obj.nPoint+1:obj.nPoint+appendDataLength) = timeStamp;
            
            % copy from previous flag, keep status unchanged
%             if obj.nPoint>0
%                 if obj.dataFlag(obj.nPoint)
%                     flag = ones(1,appendDataLength)*obj.dataFlag(obj.nPoint);
%                 else
%                     flag = zeros(1,appendDataLength);
%                 end
%             else
%                 flag = zeros(1,appendDataLength);
%             end
%             obj.dataFlag(obj.nPoint+1:obj.nPoint+appendDataLength) = flag;
            
            obj.nPoint = obj.nPoint + appendDataLength;
        end
                
    end
    
end

