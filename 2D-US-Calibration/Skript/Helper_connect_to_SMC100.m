function [status,SMC_obj] = Helper_connect_to_SMC100(port_name)
%HELPER_CONNECT_TO_SMC100 Summary of this function goes here
%   Detailed explanation goes here

%Create and open serial-port
SMC_obj.serial_port = serial(port_name,'BaudRate',57600,'DataBits',8,'FlowControl','software','Terminator','CR/LF');
fopen(SMC_obj.serial_port);

fprintf(SMC_obj.serial_port,'1rs'); % Resetting  takes 5s
pause(6);

%Enter config mode
fprintf(SMC_obj.serial_port,'1pw1'); % Set to config mode
pause(0.2);

% Config
fprintf(SMC_obj.serial_port,'1ht0'); % Homing type MZ -> there are other types see manual
pause(0.2);
fprintf(SMC_obj.serial_port,'1qir0.7'); % Set current limit to 0.7A RMS, motor limit is 0.9A RMS
pause(0.2);
fprintf(SMC_obj.serial_port,'1qil1.5'); % Set current limit to 1.5A RMS, motor limit is 1.8A RMS
pause(0.2);
fprintf(SMC_obj.serial_port,'1sr360'); % Set rotation max
pause(0.2);
fprintf(SMC_obj.serial_port,'1sl-360'); % Set rotation min
pause(0.2);
fprintf(SMC_obj.serial_port,'1va60'); % Set speed to 60°/s, max is 80°/s
SMC_obj.speed = 60; % Save set speed for later movement time calculation 
pause(0.2);

%Exit config mode
fprintf(SMC_obj.serial_port,'1pw0'); % Set to 'non-referenced' mode
pause(0.8);

fprintf(SMC_obj.serial_port,'1or'); % Homing
pause(0.8);
fprintf(SMC_obj.serial_port,'1pa0'); % Move to zero deg
pause(0.8);

%Query for position 
fprintf(SMC_obj.serial_port,'1tp?'); % get position command
str = fscanf(SMC_obj.serial_port,'%s'); % read the return data
SMC_obj.last_position = sscanf(str,'1TP%f'); % Extract position from string

if SMC_obj.last_position ~= 0 %If zero not reached 
    t_delay = abs(SMC_obj.last_position)/SMC_obj.speed; % Estimate movement time
    pause(t_delay + 5); % Wait for move to finish

    %Read position again
    fprintf(SMC_obj.serial_port,'1tp?'); 
    str = fscanf(SMC_obj.serial_port,'%s'); 
    SMC_obj.last_position = sscanf(str,'1TP%f'); 

    if abs(SMC_obj.last_position) > 0.1 % Movment failed return error
        status = 1;
    else
        status = 0; % Init finished without error
    end
else
    status = 0; % Init finished without error
end


end