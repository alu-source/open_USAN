function [status,vel] = Helper_read_US(ser_obj)
%HELPER_READ_US Summary of this function goes here
%   Detailed explanation goes here
if(ser_obj.BytesAvailable > 0)
    ret = fread(ser_obj, ser_obj.BytesAvailable);
end
fwrite(ser_obj,'-c0');
[status, vel] = Helper_float_packet_receive(ser_obj,2);

if isnan(vel)
    status = 1;
end

end

