function [status, data] = Helper_float_packet_receive(com,size)
%HELPER_FLOAT_PACKET_SEND Summary of this function goes here
%   Detailed explanation goes here
ret = fread(com,size+1,'float');

if length(ret) == size+1
    crc_packet = typecast(cast(ret(size + 1),'single'),'uint32');
    crc_data = crc32(cast(ret(1:size),'single'));
    if crc_data == crc_packet
        status = 0;
        data(1:size) = ret(1:size);
    else
        status = 1;
        data(1:size) = 0;
    end
else
    status = 1;
    data(1:size) = 0;
end
end

