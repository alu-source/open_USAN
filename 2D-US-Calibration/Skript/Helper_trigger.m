function [status] = Helper_trigger(COM)
%HELPER_TRIGGER Summary of this function goes here
%   Detailed explanation goes here
ret = query(COM,'-TR');
ret = convertCharsToStrings(ret(1:end-2));
if ret ~= "ACK"
    status = 1;
else
   status = 0;
end

end

