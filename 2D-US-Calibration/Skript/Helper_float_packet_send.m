function [status] = Helper_float_packet_send(com,data)
%HELPER_FLOAT_PACKET_SEND Summary of this function goes here
%   Detailed explanation goes here

%Reduce double to float !
data = single(data);

%Calc CRC
crc_data = crc32(data);

%Send data 
fwrite(com,data,'single');
fwrite(com,crc_data,'uint32');
ret = fscanf(com, '%s');
ret = convertCharsToStrings(ret(1:end-1));

if ret ~= "TRU"
    fwrite(com,[data,crc_data],'uint32');
    ret = fscanf(com, '%s');
    ret = convertCharsToStrings(ret(1:end-1));
    if ret ~= "TRU"
        status = 1;
    else
        status = 0;
    end

else
    status = 0;
end
end

