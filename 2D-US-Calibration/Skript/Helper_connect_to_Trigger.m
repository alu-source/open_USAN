function [status, ser_obj] = Helper_connect_to_Trigger(port)
%HELPER_CONNECT_TO_TRIGGER Summary of this function goes here
%   Detailed explanation goes here



ser_obj = serial(port,'BaudRate',9600);
fopen(ser_obj);

%Check connection should return 
%ret = query(ser_obj, '-Tr');
for i = 1:5
    fwrite(ser_obj,'-Pp','char')
    pause(1)
end
ret = fscanf(ser_obj);
ret = convertCharsToStrings(ret(1:end-2));
if ret ~= "Trigger_board"
    status = 1;
else 
    status = 0;
end
%fclose(ser_obj);
end

