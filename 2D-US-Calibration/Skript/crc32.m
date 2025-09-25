function crc = crc32(data)

%crc32   Computes the CRC-32 checksum value of a byte vector.
%--------------------------------------------------------------------------
%   CRC = crc32(DATA) computes the CRC-32 checksum value of the data stored
%   in vector DATA. The elements of DATA are interpreted as unsigned bytes
%   (uint8). The result is an unsigned 32-bit integer (uint32). Polynomial
%   bit positions have been reversed, and the algorithm modified, in order
%   to improve performance.

%   Version:    1.00
%   Programmer: Costas Vlachoss
%   Date:       23-Dec-2014

% Initialize variables
crc  = uint32(hex2dec('FFFFFFFF'));
poly = uint32(hex2dec('EDB88320'));%(hex2dec('EDB88320'));
%data = uint32(data);
data = typecast(data,'uint8');
% Compute CRC-32 value
for i = 1:length(data)
    crc = bitxor(crc,uint32(data(i)));
    for j = 1:8
        mask = bitcmp(bitand(crc,uint32(1)));
        if mask == intmax('uint32')
           mask = uint32(0);
        else
            mask =uint32(mask+1);
        end
        crc = bitxor(bitshift(crc,-1),bitand(poly,mask));
    end
end
crc = bitcmp(crc);

% function CRC = crc32(Packet)
% 
% BinaryPacket = hex2bin(Packet);
% 
% 
% %Poly: 'z^32 + z^26 + z^23 + z^22 + z^16 + z^12 + z^11 + z^10 + z^8 + z^7 + z^5 + z^4 + z^2 + z + 1'
% poly = [32,26,23,22,16,12,11,10,8,7,5,4,2,1,0];
% 
% BinaryPacket = [not(BinaryPacket(1:32));BinaryPacket(33:end)];
% crcGen = comm.CRCGenerator(...
%     'Polynomial', poly, ...
%     'InitialConditions', 0, ...
%     'ReflectInputBytes', true, ...  
%     'ReflectChecksums', true, ...    
%     'DirectMethod', false, ...
%     'FinalXOR', 1);
% seq = crcGen(BinaryPacket); 
% csNondirect = seq(end-31:end);
% 
% 
% CRC = dec2hex(bin2dec(num2str(csNondirect)'));
% 
% end