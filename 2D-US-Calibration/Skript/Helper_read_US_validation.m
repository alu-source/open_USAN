function [status, vel_raw, vel_corr] = Helper_read_US_validation(ser_obj)
%HELPER_READ_US Summary of this function goes here
%   Detailed explanation goes here

% Clear Buffer
if(ser_obj.BytesAvailable > 0)
    ret = fread(ser_obj, ser_obj.BytesAvailable);
end
fwrite(ser_obj,'-c4');
vel_raw = zeros(100,2);
vel_corr = zeros(100,2);

i = 1;
error = 0;

while i + error <= 100 && error < 10
    [status, vel] = Helper_float_packet_receive(ser_obj,4);

    if(status == 0) % Check if transmission was successful  
        vel_raw(i,:) = vel(1:2);
        vel_corr(i,:) = vel(3:4);
        i = i + 1; % Increment data index and copy to buffer
    else
        error = error + 1; % increase error count
    end
end

if sum(isnan(vel(:))) > 0 || error == 10 % If vel is nan or transmission error count was reached 
    status = 1;
end

end

