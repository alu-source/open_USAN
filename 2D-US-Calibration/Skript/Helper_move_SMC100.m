function [status,SMC_obj] = Helper_move_SMC100(SMC_obj, ang)

%Write new position
msg = sprintf('1pa%d',ang);
fprintf(SMC_obj.serial_port,msg);

%Estimate travel time
t_delay = abs(SMC_obj.last_position - ang) / SMC_obj.speed;
t_magic = 0.2;
pause(t_delay + t_magic);

%Read current postion from SMC
fprintf(SMC_obj.serial_port,'1tp?');
str = fscanf(SMC_obj.serial_port,'%s');
SMC_obj.last_position = sscanf(str,'1TP%f');


%Raise error if ang error greater 0.1°
if(abs(SMC_obj.last_position - ang) > 0.1)
    status = 1;
else 
    status = 0;
end
end

