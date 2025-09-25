function [status, ser_obj] = Helper_connect_US(com_name)
%HELPER_CONNECT_US Summary of this function goes here
%   Detailed explanation goes here

ser_obj = serial(com_name,'BaudRate',1000000);
ser_obj.InputBufferSize  = 1024;
ser_obj.OutputBufferSize = 1024;

fopen(ser_obj);

%Check connection should return 
for i = 1:5
    fwrite(ser_obj,'-P','char');
    pause(.25);
end
ret = fscanf(ser_obj);
ret = convertCharsToStrings(ret(1:end-1));
if ret ~= "USAN"
    status = 1;
else 
    status = 0;
end

pause(1);
if ser_obj.BytesAvailable > 0
    ret = fread(ser_obj, ser_obj.BytesAvailable);
end

%Set to calibration mode
ret = query(ser_obj,'-c0');
ret = convertCharsToStrings(ret(1:end-1));
if ret ~= "Cali_mode" && ret ~= " Cali_mode"
        pause(1);
        ret = query(ser_obj,'-c0');
        ret = convertCharsToStrings(ret(1:end-1));
        if ret ~= "Cali_mode" && ret ~= " Cali_mode"
            status = status + 1;
        end
end

%Read Sensor ID 

end

