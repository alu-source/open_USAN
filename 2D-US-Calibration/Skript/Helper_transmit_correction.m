function [status] = Helper_transmit_correction(serial_obj, correction_matrix, write_flag)
%HELPER_TRANSMITT_CORRECTION Summary of this function goes here
%   Detailed explanation goes here

status = 0;

%Flush serial
while(serial_obj.BytesAvailable > 0)
    ret = fread(serial_obj, serial_obj.BytesAvailable);
    pause(0.01);
end

%Set in to calibration parsing mode and wait for confirmation of the
%sensor!

ret = query(serial_obj,'-c1');
ret = convertCharsToStrings(ret(1:end-1));

if ret ~= "Listening"
    status = 1;
else

    % first transmit X
    for i = 1:length(correction_matrix)
        [status] = Helper_float_packet_send(serial_obj, squeeze(correction_matrix(i,:,1)));
        if status ~= 0 % Transmittion error
            [status] = Helper_float_packet_send(serial_obj, squeeze(correction_matrix(i,:,1)));
            if status ~= 0
                i = 9999;
            end
        end
    end

    if status == 0
        % second transmit Y
        for i = 1:length(correction_matrix)
            [status] = Helper_float_packet_send(serial_obj, squeeze(correction_matrix(i,:,2)));
            if status ~= 0 % Transmittion error
                [status] = Helper_float_packet_send(serial_obj, squeeze(correction_matrix(i,:,2)));
                if status ~= 0
                    i = 9999;
                end
            end
        end
    end
    % If transmittion is ok and wirte flag set, give write command ! 
    if status == 0 && write_flag
        fwrite(serial_obj,'-w', 'char');
        fwrite(serial_obj,'-w', 'char');
    end
end
end
