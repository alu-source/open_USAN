clc;
%% Setup

% Clear old var exept the uart stuff
clearvars -except SMC US_port TR_port

% Configure calibration
stepp_angle = 3; % Default: 5
% vel_list = [35]; % Default: [1,5,10,20,30,45]
vel_list = [5,10,20,30,35]; % Default: [1,5,10,20,30,45]

% User Input

% Storage point fixed to work dir / calib_data !
Sensor_ID = inputdlg('Enter Sensor ID','Sensor Calibration', [1 50]);
default_path = ['Calibration_data_ID',Sensor_ID{1}];
calibration_path = inputdlg('Enter storage folder name','Sensor Calibration', [1 50], {default_path});
workspace_path = pwd;
calibration_path = [workspace_path,'/sensor_calib_data/',calibration_path{1}];

% Check if folder exists and warn accordingly
if ~exist(calibration_path, 'dir')
    mkdir(calibration_path);
else
    warndlg('Folder already exists, the data might be overwritten!','Warning');
end
fprintf('Absolut path: %s \n',calibration_path);

%Com port Setup
if ~exist('SMC') % Check if Com is already connected
    SMC_port = 0;
    status = 1;
    while status == 1
        com_ports = seriallist();
        fn = com_ports;
        [indx,tf] = listdlg('PromptString',{'Select the Turntable COM-Port.'},'SelectionMode','single','ListString',fn);

        SMC_port = com_ports(indx);
        [status,SMC] = Helper_connect_to_SMC100(SMC_port);

        if status == 1
            f = warndlg('Turntable not responding wrong Port','Warning');
        end
    end
end

if ~exist('US_port') % Check if Com is already connected
    US_port = 0;
    status = 2;
    while status == 2
        com_ports = seriallist();
        fn = com_ports;
        [indx,tf] = listdlg('PromptString',{'Select the US Anemometer COM-Port.'},'SelectionMode','single','ListString',fn);

        US_port = com_ports(indx);
        [status,US_port] = Helper_connect_US(US_port);
        

        if status == 2
            f = warndlg('US Anemometer not responding wrong Port','Warning');
        end
    end
end





%% Start the measurement sequence
n_vel = length(vel_list);
n_ang = 360 / stepp_angle;
ang_list = [-180:stepp_angle:180];
ang_list(end) = [];
% n_ang = 1;
% ang_list = 130;

% Storage 
raw_buffer_x = zeros(n_vel, n_ang);
raw_buffer_y = zeros(n_vel, n_ang);

raw_ref = zeros(1,n_vel);

for vi = 1:n_vel
    %Requesting user to change speed and enter meas. speed 
    msg_string = sprintf('Set speed to: %f m/s and enter actual achived speed',vel_list(vi));
    vel_actual = inputdlg(msg_string,'Sensor Calibration', [1 50]);
    raw_ref(vi) =  str2num(vel_actual{1});

    for ai = 1:n_ang % Because -180 and + 180 are the same angle
        % Setting turntable to the postion 
        [status , SMC] = Helper_move_SMC100(SMC,ang_list(ai));
        if status == 1
            % Retry
			pause(0.2)
            [status , SMC] = Helper_move_SMC100(SMC,ai);
            if status == 1
                 f = warndlg('Turntable failed to move !','Error');
            end
        end
		
		pause(0.2) % Wait for airflow to be stable

        [status,v] = Helper_read_US(US_port);
        if status ~= 0
            [status,v] = Helper_read_US(US_port);
                if status ~= 0
                    f = warndlg('US Communication failed!','Error');
                    % Error handling !!!
                end
        end
        raw_buffer_x(vi,ai) = v(1);
        raw_buffer_y(vi,ai) = v(2);

        % Display current angle in Command Window
        fprintf('Measurement at %3.2f completed, %2.2f %% done \n',ang_list(ai),((n_ang * (vi - 1) + ai)/(n_vel * n_ang)) * 100);
    end
end

% Plot raw data 
fig_raw_data = figure();
v_abs = sqrt(raw_buffer_x.^2 + raw_buffer_y.^2);
a_abs =  atan2d(raw_buffer_x,raw_buffer_y) - 180; % Fixen ?

for i = 1:n_ang
    v_ref_list(1:n_vel,i) = raw_ref;
end
for i = 1:n_vel
    a_ref_list(i,1:n_ang) = ang_list;
end

v_diff = v_ref_list - v_abs;
a_diff = abs(a_ref_list - a_abs);

a_wrap_list = find(a_diff > 180);
a_diff(a_wrap_list) = abs(a_diff(a_wrap_list) - 360);

subplot(1,2,1);
polarplot(deg2rad(a_ref_list)',v_abs');
rlim([0,50]);
%ADD radial axis scale 
%This may break the script in older versions of matlab
%rlim([0, 50]);
%rticks([0 5 10 20 30 40 50 60])
%rticklabels({'0','5','10','20','30','40','','m/s'});
%set(gca,'RAxisLocation',157.5);
%thetaticks([0 45 90 135 157.5 180 225 270 315])
%thetaticklabels(["0°", '45°', '90°', '135°', 'v_{abs}, m/s', '180°', '225°', '270°', '315°']);


title(['v_{abs, meas} across inflow angle']);

subplot(1,2,2);
yyaxis left;
plot(a_ref_list(:,1:n_ang)',v_diff(:,1:n_ang)');
ylabel('v_{abs, meas} abs error, m/s');
ylim([0,15]);
xlim([-180, 65]);

yyaxis right;
plot(a_ref_list(:,1:n_ang)',a_diff(:,1:n_ang)');
ylabel('Angle abs error, deg');
ylim([0,15]);
xlim([-180, 175]);

xlabel('Inflow angle, deg');

title('Angle and v_{abs} measurement error across inflow angle');
fig_raw_data.Position = [100 100 923 400];

% Save the measured data so it can't be lost!
% Old storage option
sensor_struct.raw.raw_buffer_x = raw_buffer_x;
sensor_struct.raw.raw_buffer_y = raw_buffer_y;
sensor_struct.raw.raw_ref = raw_ref;
sensor_struct.raw.ang_list = ang_list;
% sensor_struct.raw.fig_raw_data = fig_raw_data;
save(calibration_path ,'sensor_struct');

% New storage option
sensor_raw_struct.raw_buffer_x = raw_buffer_x;
sensor_raw_struct.raw_buffer_y = raw_buffer_y;
sensor_raw_struct.raw_ref = raw_ref;
sensor_raw_struct.ang_list = ang_list;
% sensor_raw_struct.fig_raw_data = fig_raw_data;
file_path = [calibration_path,'/raw_data'];
print(fig_raw_data,[file_path,'.png'],'-dpng','-r300');
savefig(fig_raw_data,[file_path,'.fig']);
save([file_path,'.mat'] ,'sensor_raw_struct');


% Set sensor to 0 position
[status , SMC] = Helper_move_SMC100(SMC,0);


%% Calculate the Correction

% Check raw data for errors and correct if necessary
dump = questdlg('Do you want to check raw data for errors ?','Error check','Yes','No','Yes');
if strcmp(dump,'Yes')
    [corr_x, corr_y, ~]= Helper_correct_calib_data(sensor_struct.raw);
    sensor_struct.raw.raw_buffer_x = corr_x;
    sensor_struct.raw.raw_buffer_y = corr_y;
end

tic
[lookup_matrix, error_matrix, iter_matrix] = Iterative_point_interpolation(sensor_struct.raw);
% [lookup_matrix, error_matrix, iter_matrix] =
% Iterative_point_interpolation(sensor_raw_struct); 
% for use with new data storage

toc

% Calculate the relevant error values
n_stepps = 181;
v_stepp = 0.45;
core_error = 0;
for ix = 1:n_stepps
    for iy = 1:n_stepps
        vx = -(n_stepps - 1)/2 * v_stepp +  v_stepp*(ix-1);
        vy = -(n_stepps - 1)/2 * v_stepp +  v_stepp*(iy-1);
        if (sqrt(vx^2 + vy^2) < 35)
            core_error(end + 1) = error_matrix(ix,iy);
        end
    end
end


max(core_error);
mean(core_error);
median(core_error);

% Display correction
disp_matrix = lookup_matrix;
for i = 1:n_stepps
    vx = -(n_stepps - 1)/2 * v_stepp +  v_stepp*(i-1);
    vy = -(n_stepps - 1)/2 * v_stepp +  v_stepp*(i-1);
    disp_matrix(:,i,1) = disp_matrix(:,i,1) + vx;
    disp_matrix(i,:,2) = disp_matrix(i,:,2) - vy;
end

% fig_corr = figure();
% 
% subplot(1,2,1);
% surf([-40.5:0.45:40.5],[-40.5:0.45:40.5],disp_matrix(:,:,1));
% xlabel('v_{x, meas}');
% ylabel('v_{y, meas}');
% title('X_corr Defecit');
% 
% subplot(1,2,2);
% surf([-40.5:0.45:40.5],[-40.5:0.45:40.5],disp_matrix(:,:,2));
% xlabel('v_{x, meas}');
% ylabel('v_{y, meas}');
% title('Y_corr Defecit');


% Point list 
for i = 1:length(sensor_struct.raw.raw_ref)   
    P((i-1) * n_ang + 1: i * n_ang, 1:2) = [sensor_struct.raw.raw_buffer_x(i,:)', sensor_struct.raw.raw_buffer_y(i,:)'];
    P_ang_ref((i-1) * n_ang + 1: i * n_ang) = sensor_struct.raw.ang_list';
    P_v_ref((i-1) * n_ang + 1: i * n_ang) = sensor_struct.raw.raw_ref(i);
end



v_corr = Helper_apply_correction_matrix(lookup_matrix,P);

v_abs_c = sqrt( v_corr(:,1).^2 + v_corr(:,2).^2);

v_ang_c = atan2d (-v_corr(:,1),-v_corr(:,2));



fig_corr = figure();
% Plot of raw data with correction applied 
subplot(2,2,[1,3]);
polarplot(deg2rad(v_ang_c), v_abs_c,'LineStyle','none','Marker','o');
title("Calibration Selftest");
rlim([0,50]);

a_error = abs(P_ang_ref - v_ang_c');
a_list = find(a_error > 300);
a_error(a_list) = abs(a_error(a_list) - 360); 
v_error = abs(P_v_ref - v_abs_c');

fprintf('Angle error, max = %f deg, mean = %f deg\n',max(a_error),mean(a_error));
fprintf('Vel_abs error, max = %f m/s, mean = %f m/s\n',max(v_error),mean(v_error));

v_error_over_vref = 0;
for i = 1:length(sensor_struct.raw.raw_ref)   
    v_error_over_vref_mean(i) = mean(v_error((i-1) * n_ang + 1: i * n_ang));
    v_error_over_vref_max(i) = max(v_error((i-1) * n_ang + 1: i * n_ang));

end

% Plot error over V_ref -> a small increase in error can be expected
subplot(2,2,[2]);
plot(sensor_struct.raw.raw_ref, v_error_over_vref_mean,'Marker','+');
hold on;
plot(sensor_struct.raw.raw_ref, v_error_over_vref_max,'Marker','o');
title('V_{corr} error across V_{ref}');
ylabel('Error, m/s');
xlabel('V_{ref}, m/s');
legend('mean error','max error');
hold off;

% Plot overall errors as a table
hAx = subplot(2,2,[4]);
error_name = {'max vel error, m/s'; 'mean vel error, m/s'; 'max angle error, deg'; 'mean angle error, deg'};
error_values = [max(v_error); mean(v_error); max(a_error); mean(a_error)];
T = table(error_values,'RowNames',error_name);
hUI=uitable('Data',T{:,:},'ColumnName',T.Properties.VariableNames,...           
'RowName',T.Properties.RowNames,'Units', 'Normalized', 'Position',hAx.Position);% set table on top
delete(hAx)

fig_corr.Position = [100 100 923 400];

% Warn user if errors are to big
if(max(v_error) > 0.5 || mean(v_error) > 0.1 || max(a_error) > 7.2 || mean(a_error) > 0.5)
    warndlg('Warning correction error is unusually high, this may lead to poor sensor performance !','Calibration Selftest Failed !');
end


% Storage
% Old version
file_path = [calibration_path,'.mat'];
load(file_path,'sensor_struct');
sensor_struct.calib.lookup_matrix = lookup_matrix;
sensor_struct.calib.fig_corr = fig_corr;
save(file_path,'sensor_struct');

% New storage option
sensor_calib_struct.lookup_matrix = lookup_matrix;
sensor_calib_struct.fig_corr = fig_corr;
file_path = [calibration_path,'/calib_data'];
print(fig_corr,[file_path,'.png'],'-dpng','-r300');
savefig(fig_corr,[file_path,'.fig']);
save([file_path,'.mat'] ,'sensor_calib_struct');


%% Transfer  calibration
% Give user the opportunity to abort 
trans_flag = false;
dump = questdlg('Do you want to transfer the correction to the Sensor ?','Save','Yes','No','Yes');
if strcmp(dump,'Yes')
    trans_flag = true;
end

if trans_flag
    % Should the correction be saved ?!
    save_flag = false;
    dump = questdlg('Save after transferring ?','Save','Yes','No','Yes');
	if strcmp(dump,'Yes')
        save_flag = true;
    end

    % Send to MCU
    fprintf('Start transferring ...\n');
    while(US_port.BytesAvailable > 0) % Clean up rx buffer
        ret = fread(US_port, US_port.BytesAvailable);
        pause(0.01);
    end
    [status] = Helper_transmit_correction(US_port, lookup_matrix, save_flag); %Does the actual transmitting

    if status == 0 % Give feedback in console
        fprintf('Transfer finished\n');
    else
        fprintf('Transfer finished with error value\n');
    end
end


%% The Validation part !!!
valid_flag = false;
dump = questdlg('Do you want to Validate the correction?','Validation','Yes','No','Yes');
if strcmp(dump,'Yes')
    valid_flag = true;
end

if valid_flag

    % Use trigger !
    tr_flag = false;
    dump = questdlg('Use a trigger for external recording?','Trigger setup','Yes','No','No');
    if strcmp(dump,'Yes')
        tr_flag = true;
    end

    TR_port = 0;
    if tr_flag
        status = 1;
        while status == 1
            com_ports = seriallist();
            fn = com_ports;
            [indx,tf] = listdlg('PromptString',{'Select the Trigger COM-Port.'},'SelectionMode','single','ListString',fn);

            TR_port = com_ports(indx);
            [status,TR_port] = Helper_connect_to_Trigger(TR_port);

            if status == 1
                f = warndlg('Trigger not responding wrong Port','Warning');
            end

            % Ping and DAC wiggel !
            if(status == 0) 
                Helper_trigger(TR_port);
                fwrite(US_port,'-P-P','char');
                ret = fscanf(US_port);
                ret = convertCharsToStrings(ret(1:end-1));
%                 if ret ~= 'USAN'
%                     status = 1;
%                 else
%                     status = 0;
%                 end
            end
        end
    end

    % Number of speeds
    msg_string = sprintf('At how many speeds should the validation take place?');
    ret = inputdlg(msg_string,'Sensor Calibration', [1 50]);
    n_validation_v_ref = str2num(ret{1});

    % Number of angles
    msg_string = sprintf('At how many angles should the validation take place?');
    ret = inputdlg(msg_string,'Sensor Calibration', [1 50]);
    n_validation_ang_ref = str2num(ret{1});

    % Create list of angles
    ang_ref_list = zeros(n_validation_v_ref, n_validation_ang_ref);

    % -> Randome points sorted from small to big
    for i = 1:n_validation_v_ref
        ang_ref_list(i,:) = sort(180 - rand(1,n_validation_ang_ref) * 360);
    end

    % Start the measurement
    raw_buffer_x = zeros(n_validation_v_ref, n_validation_ang_ref,100);
    raw_buffer_y = zeros(n_validation_v_ref, n_validation_ang_ref,100);
    corr_buffer_x = zeros(n_validation_v_ref, n_validation_ang_ref,100);
    corr_buffer_y = zeros(n_validation_v_ref, n_validation_ang_ref,100);

    for vi = 1:n_validation_v_ref
        % Requesting user to change speed and enter meas. speed
        msg_string = sprintf('Enter speed');
        vel_actual = inputdlg(msg_string,'Sensor Calibration', [1 50]);
        raw_ref(vi) =  str2num(vel_actual{1});

        for ai = 1:n_validation_ang_ref %Because -180 and + 180 are the same angle
            % Setting Turntable to the postion
            [status , SMC] = Helper_move_SMC100(SMC,ang_ref_list(vi,ai));
            if status == 1
                % Retry
				pause(0.2)
                [status , SMC] = Helper_move_SMC100(SMC,ai);
                if status == 1
                    f = warndlg('Turntable failed to move !','Error');
                end
            end

            % Measure
            if tr_flag == 1
                Helper_trigger(TR_port);
            end
            [status,v_raw, v_corr] = Helper_read_US_validation(US_port);
            if status ~= 0
                [status,v_raw, v_corr] = Helper_read_US_validation(US_port);
                if status ~= 0
                    f = warndlg('US Communication failed!','Error');
                    % Error handling !!!
                end
            end
            raw_buffer_x(vi,ai,:) = v_raw(:,1);
            raw_buffer_y(vi,ai,:) = v_raw(:,2);
            corr_buffer_x(vi,ai,:) = v_corr(:,1);
            corr_buffer_y(vi,ai,:) = v_corr(:,2);
            %Display Current Angle in Command Window
            fprintf('Measurement at %3.2f completed, %2.2f %% done \n',ang_ref_list(vi,ai),((n_validation_ang_ref * (vi - 1) + ai)/(n_validation_v_ref * n_validation_ang_ref)) * 100);
        end
    end
    % Give the result of the validation  
end

%% Plot
% Calc Error and Plot error stat
vel_corr_mean_error = zeros(n_validation_v_ref, n_validation_ang_ref);
vel_corr_std_error = zeros(n_validation_v_ref, n_validation_ang_ref);
ang_corr_mean_error = zeros(n_validation_v_ref, n_validation_ang_ref);
ang_corr_std_error = zeros(n_validation_v_ref, n_validation_ang_ref);
corr_meas_error = zeros(n_validation_v_ref, n_validation_ang_ref);
corr_trans_error = zeros(n_validation_v_ref, n_validation_ang_ref);


for vi = 1:n_validation_v_ref
    for ai = 1:n_validation_ang_ref
        error_x = find(squeeze(corr_buffer_x(vi,ai,:)) == 0);
        error_y = find(squeeze(corr_buffer_y(vi,ai,:)) == 0);
        error_list = sort(unique([error_x;error_y])); % Sorted and without duplicates
        corr_meas_error(vi,ai) = length(error_list);
        working_buffer_x = squeeze(corr_buffer_x(vi,ai,:));
        working_buffer_y = squeeze(corr_buffer_y(vi,ai,:));

        if(length(error_list) > 0) % Delete error values form data
            working_buffer_x(error_list) = [];
            working_buffer_y(error_list) = [];

          % Don't count com errors
            n_errors = length(error_list);
            com_error_VR = [(100 - n_errors + 1):100]';
            inter_error_list = error_list - com_error_VR;
            com_error_list = find(inter_error_list == 0);
            inter_error_list(com_error_list) = [];
            corr_meas_error(vi,ai) = length(inter_error_list);
            corr_trans_error(vi,ai) = length(com_error_list);
        else
            corr_meas_error(vi,ai) = 0;
            corr_trans_error(vi,ai) = 0;
        end
        
        % Velocity error
        v_abs = sqrt(working_buffer_x.^2 + working_buffer_y.^2);
        v_abs_error = abs(mean(v_abs) - raw_ref(vi));

        vel_corr_mean_error(vi,ai) = v_abs_error;
        vel_corr_std_error(vi,ai) = std(v_abs);

        % Angle error
        ang_abs = atan2d(working_buffer_x, working_buffer_y) - 180;
        ang_diff = abs(ang_abs - ang_ref_list(vi,ai));
        ang_wrap_list = find(ang_diff > 300);
        ang_diff(ang_wrap_list) = abs(ang_diff(ang_wrap_list) - 360);

        ang_corr_mean_error(vi,ai) = mean(ang_diff);
        ang_corr_std_error(vi,ai) = std(ang_diff);
    end
end


% Start  plotting
fig_validation = figure();

% First polar plot with all meas 
subplot(2,4,[1,2,5,6]);
v_abs = sqrt(corr_buffer_x.^2 + corr_buffer_y.^2);
ang = atan2d(corr_buffer_x, corr_buffer_y) - 180;
ang_wrap_list = find(ang < 0);
ang(ang_wrap_list) = abs(ang(ang_wrap_list) + 360);

for i = 1:n_validation_v_ref
    p_1 = polarplot(deg2rad(squeeze(ang(i,:,:))),squeeze(v_abs(i,:,:)),'LineStyle','none', 'Marker','*','Color','b');
    if i == 1
        hold on;
    end
    raw_buff(1:n_validation_ang_ref) = raw_ref(i);
    p_2 = polarplot(deg2rad(ang_ref_list(i,:)'),raw_buff','LineStyle','none', 'Marker','o','MarkerSize',10,'Color','r');
    legend([p_1(1), p_2],'Measured point','Reference point','Location','southeast');
end

hold off;
title("Validation");
rlim([0,50]);

% V_error over V
subplot(2,4,[3]);
std_vec_mean = mean(vel_corr_std_error,2);
std_vec_max = max(vel_corr_std_error,[],2);
mean_vec_mean = mean(vel_corr_mean_error,2);
mean_vec_max = max(vel_corr_mean_error,[],2);

vel_max_mean= max(mean_vec_max);
vel_mean_mean = mean(vel_corr_mean_error(:));
vel_max_std = max(std_vec_max);
vel_mean_std = mean(vel_corr_std_error(:));

% Get occurence of the biggest error
buf_vel_idx = find(std_vec_max == vel_max_std); % Find Velocity
vel_max_std_at_vel = raw_ref(buf_vel_idx);
buf_ang_idx = find(vel_corr_std_error(buf_vel_idx,:) == vel_max_std); % Find Angle
vel_max_std_at_ang = ang_ref_list(buf_vel_idx,buf_ang_idx);

buf_vel_idx = find(mean_vec_max == vel_max_mean); % Find Velocity
vel_max_mean_at_vel = raw_ref(buf_vel_idx);
buf_ang_idx = find(vel_corr_mean_error(buf_vel_idx,:) == vel_max_mean); % Find Angle
vel_max_mean_at_ang = ang_ref_list(buf_vel_idx,buf_ang_idx);



yyaxis left;
plot(raw_ref(1:n_validation_v_ref),mean_vec_mean,'Marker','*');
hold on;
plot(raw_ref(1:n_validation_v_ref),mean_vec_max,'Marker','+');
hold off;
legend('mean', 'max');
ylabel('V_{abs} mean error, m/s');
ylim([0,2.5]);
xlim([0,50]);

yyaxis right;
plot(raw_ref(1:n_validation_v_ref),std_vec_mean,'Marker','*');
hold on;
plot(raw_ref(1:n_validation_v_ref),std_vec_max,'Marker','+');
hold off;
legend('mean', 'max');
ylabel('V_{abs} std, m/s');
ylim([0,1]);
xlim([0,50]);

grid on;
title('V_{abs} errors across V_{ref}');
xlabel('V_{ref}');


% Same for angle 
subplot(2,4,[4]);
std_vec_mean = mean(ang_corr_std_error,2);
std_vec_max = max(ang_corr_std_error,[],2);
mean_vec_mean = mean(ang_corr_mean_error,2);
mean_vec_max = max(ang_corr_mean_error,[],2);

ang_max_mean= max(mean_vec_max);
ang_mean_mean = mean(ang_corr_mean_error(:));
ang_max_std = max(std_vec_max);
ang_mean_std = mean(ang_corr_std_error(:));

% Get occurence of the biggest error
buf_vel_idx = find(std_vec_max == ang_max_std); % Find Velocity
ang_max_std_at_vel = raw_ref(buf_vel_idx);
buf_ang_idx = find(ang_corr_std_error(buf_vel_idx,:) == ang_max_std); % Find Angle
ang_max_std_at_ang = ang_ref_list(buf_vel_idx,buf_ang_idx);

buf_vel_idx = find(mean_vec_max == ang_max_mean); % Find Velocity
ang_max_mean_at_vel = raw_ref(buf_vel_idx);
buf_ang_idx = find(ang_corr_mean_error(buf_vel_idx,:) == ang_max_mean); % Find Angle
ang_max_mean_at_ang = ang_ref_list(buf_vel_idx,buf_ang_idx);


yyaxis left;
plot(raw_ref(1:n_validation_v_ref),mean_vec_mean,'Marker','*');
hold on;
plot(raw_ref(1:n_validation_v_ref),mean_vec_max,'Marker','+');
hold off;
legend('mean', 'max');
ylabel('Ang mean error, deg');
ylim([0,10]);
xlim([0,50]);

yyaxis right;
plot(raw_ref(1:n_validation_v_ref),std_vec_mean,'Marker','*');
hold on;
plot(raw_ref(1:n_validation_v_ref),std_vec_max,'Marker','+');
hold off;
legend('mean', 'max');
ylabel('Ang std, deg');
ylim([0,3]);
xlim([0,50]);

xlabel('V_{ref}');
title('Angle errors across V_{ref}');
grid on;

% Plot a tabel with over all errors
hAx = subplot(2,4,[7,8]);
error_name = {'max vel error, m/s'; 'mean vel error, m/s'; 'max angle error, deg'; 'mean angle error, deg';'max std vel, m/s';'mean std vel, m/s';'max std ang, deg';'mean std ang, deg';'Overall meas errors';'Overall trans errors'};
error_values = [vel_max_mean; vel_mean_mean; ang_max_mean; ang_mean_mean; vel_max_std; vel_mean_std; ang_max_std; ang_mean_std; sum(corr_meas_error(:)); sum(corr_trans_error(:))];
error_at_velocity = [vel_max_mean_at_vel; nan; ang_max_mean_at_vel; nan; vel_max_std_at_vel; nan; ang_max_std_at_vel; nan; nan; nan;];
error_at_angle =    [vel_max_mean_at_ang; nan; ang_max_mean_at_ang; nan; vel_max_std_at_ang; nan; ang_max_std_at_ang; nan; nan; nan;];
T = table(error_values,error_at_velocity, error_at_angle,'RowNames',error_name);
hUI=uitable('Data',T{:,:},'ColumnName',T.Properties.VariableNames,...           
'RowName',T.Properties.RowNames,'Units', 'Normalized', 'Position',hAx.Position);% set table on top
delete(hAx)
fig_validation.Position = [100 100 1400 760];
set(findall(gcf,'-property','FontSize'),'FontSize',12);

%Storage
% Old data
file_path = [calibration_path,'.mat'];
load(file_path,'sensor_struct');
sensor_struct.calib.corr_buffer_x = corr_buffer_x;
sensor_struct.calib.corr_buffer_y = corr_buffer_y;
sensor_struct.calib.raw_buffer_x = raw_buffer_x;
sensor_struct.calib.raw_buffer_y = raw_buffer_y;
sensor_struct.calib.ang_ref_list = ang_ref_list;
sensor_struct.calib.vel_ref = raw_ref;
sensor_struct.calib.fig_validation = fig_validation;
save(file_path,'sensor_struct');

% New storage option
sensor_valid_struct.corr_buffer_x = corr_buffer_x;
sensor_valid_struct.corr_buffer_y = corr_buffer_y;
sensor_valid_struct.raw_buffer_x = raw_buffer_x;
sensor_valid_struct.raw_buffer_y = raw_buffer_y;
sensor_valid_struct.ang_ref_list = ang_ref_list;
sensor_valid_struct.vel_ref = raw_ref;
sensor_valid_struct.fig_validation = fig_validation;

file_path = [calibration_path,'/valid_data'];
print(fig_validation,[file_path,'.png'],'-dpng','-r300');
savefig(fig_validation,[file_path,'.fig']);
save([file_path,'.mat'] ,'sensor_valid_struct');

%% Cleanup
% Close the Serialports
if tr_flag == 1
    dump = questdlg('Do you want to disconnect the Trigger board?','Validation','Yes','No','Yes');
    if strcmp(dump,'Yes')
        fclose(TR_port);
        clear TR_port
    end
end

dump = questdlg('Do you want to disconnect the rotation stage?','Validation','Yes','No','Yes');
if strcmp(dump,'Yes')
   fclose(SMC.serial_port);
   clear SMC
end

dump = questdlg('Do you want to disconnect the US-Anenometer?','Validation','Yes','No','Yes');
if strcmp(dump,'Yes')
   fclose(US_port);
   clear US_port
end