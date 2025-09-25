function [corrected_x, corrected_y, n_corrections] = Helper_correct_calib_data(sensor_obj)
corrected_x = sensor_obj.raw_buffer_x;
corrected_y = sensor_obj.raw_buffer_y;
n_corrections = 0;

n_ang = length(sensor_obj.ang_list);
n_vel = length(sensor_obj.raw_ref);

% Look at the vel defecit and check for errors ! 
x_comp = sin(deg2rad(sensor_obj.ang_list));
y_comp = -cos(deg2rad(sensor_obj.ang_list));
x_ref = sensor_obj.raw_ref' * x_comp;
y_ref = sensor_obj.raw_ref' * y_comp;

raw_ref_buffer(:,1:n_ang) =  repmat(sensor_obj.raw_ref',1,n_ang);
vel_def_x = -(sensor_obj.raw_buffer_x - x_ref);
vel_def_y = -(sensor_obj.raw_buffer_y - y_ref);
vel_def_x = vel_def_x./raw_ref_buffer;
vel_def_y = vel_def_y./raw_ref_buffer;

% vel_def_diff = vel_def_x(5,:) - vel_def_x(6,:);
% plot([-180:5:179],vel_def_diff)
% hold on 
% plot([-180,180],[-0.04,-0.04])
% plot([-180,180],[0.04,0.04])
% hold off



% Find error values
% in 3 sigma -> 99.8% should be in hear 
min_allowed_diff = 0.01;
x_error_list = struct('vel',0,'ang',0,'length',0);
y_error_list = struct('vel',0,'ang',0,'length',0);
for i = 2:n_vel
    vel_deficit_diff_x = vel_def_x(i,:) - vel_def_x(i-1,:);
    vel_deficit_diff_x = vel_deficit_diff_x - mean(vel_deficit_diff_x);
    plot(vel_deficit_diff_x);
    std_vel_deficit_diff_x = std(vel_deficit_diff_x);
    std_vel_deficit_diff_x = 0.04/3; %max([std_vel_deficit_diff_x,min_allowed_diff])
    
    hold on;
    plot([1,180],[std_vel_deficit_diff_x*3, std_vel_deficit_diff_x*3]);
    plot([1,180],[-std_vel_deficit_diff_x*3, -std_vel_deficit_diff_x*3]);
    hold off

    if(max(abs(vel_deficit_diff_x)) >  std_vel_deficit_diff_x*3)
        error_list =  find(abs(vel_deficit_diff_x) >std_vel_deficit_diff_x*3);
        j = 1;
        len = 1;
        if length(error_list) > 1
            while j < length(error_list)+1
                    %len = 1;
                    initial_idx = error_list(j);
                    if j < length(error_list) 
                         if(error_list(j + 1) == error_list(j) + 1)
                            len = len + 1;
                         else
                             x_error_list(length(x_error_list) + 1).vel = i;
                             x_error_list(length(x_error_list)).ang = initial_idx - (len - 1);
                             x_error_list(length(x_error_list)).length = len; 
                             len = 1;
                         end
                    elseif(error_list(j) == error_list(j - 1) + 1)
                           x_error_list(length(x_error_list) + 1).vel = i;
                           x_error_list(length(x_error_list)).ang = initial_idx - (len - 1);
                           x_error_list(length(x_error_list)).length = len; 
                           len = 1;
                    end
                    j = j + 1;
            end
        else
           x_error_list(length(x_error_list) + 1).vel = i;
           x_error_list(length(x_error_list)).ang = error_list;
           x_error_list(length(x_error_list)).length = 1; 
        end
    end
    
    
    vel_deficit_diff_y = vel_def_y(i,:) - vel_def_y(i-1,:);
    vel_deficit_diff_y = vel_deficit_diff_y - mean(vel_deficit_diff_y);
    plot(vel_deficit_diff_y);
    std_vel_deficit_diff_y = std(vel_deficit_diff_y);
    std_vel_deficit_diff_y = 0.04/3; %max([std_vel_deficit_diff_y,min_allowed_diff])
    
    hold on;
    plot([1,180],[std_vel_deficit_diff_y*3, std_vel_deficit_diff_y*3]);
    plot([1,180],[-std_vel_deficit_diff_y*3, -std_vel_deficit_diff_y*3]);
    hold off

    if(max(abs(vel_deficit_diff_y)) >  std_vel_deficit_diff_y*3)
        error_list =  find(abs(vel_deficit_diff_y) >std_vel_deficit_diff_y*3);
        j = 1;
        len = 1;
        if length(error_list) > 1
            while j < length(error_list)+1
                    %len = 1;
                    initial_idx = error_list(j);
                    if j < length(error_list) 
                         if(error_list(j + 1) == error_list(j) + 1)
                            len = len + 1;
                         else
                             y_error_list(length(y_error_list) + 1).vel = i;
                             y_error_list(length(y_error_list)).ang = initial_idx - (len - 1);
                             y_error_list(length(y_error_list)).length = len; 
                             len = 1;
                         end
                    elseif(error_list(j) == error_list(j - 1) + 1)
                           y_error_list(length(y_error_list) + 1).vel = i;
                           y_error_list(length(y_error_list)).ang = initial_idx - (len - 1);
                           y_error_list(length(y_error_list)).length = len; 
                           len = 1;
                    end
                    j = j + 1;
            end
        else
            y_error_list(length(y_error_list) + 1).vel = i;
            y_error_list(length(y_error_list)).ang = error_list;
            y_error_list(length(y_error_list)).length = 1; 
        end
    end
end
y_error_list(1) = [];
x_error_list(1) = [];
%% Correct errors in X

cur_error_list = x_error_list;
if ~isempty(cur_error_list) 
if(cur_error_list(1).vel ~= 0)


    cur_error_list = x_error_list;
    cur_vel_def = vel_def_x;
    cur_vel_ref = x_ref;
    cur_raw_buffer = sensor_obj.raw_buffer_x;
    cur_corr_buffer = corrected_x;

    if(cur_error_list(1).vel ~= 0)
        for i = 1:length(cur_error_list)
            if(cur_error_list(i).length == 0)
                % Simply interpolate between ranges left and right
                pp_ang = mod(cur_error_list(i).ang - 2, n_ang);
                p_ang = mod(cur_error_list(i).ang - 1, n_ang);
                ang = mod(cur_error_list(i).ang - 0, n_ang);
                f_ang = mod(cur_error_list(i).ang + 1, n_ang);
                ff_ang = mod(cur_error_list(i).ang + 2, n_ang);
                ang_list = [pp_ang, p_ang, ang, f_ang, ff_ang]; % List of velocities to plot

                ang_stepp = 360 / n_ang;
                ang_idx_list = [-2*ang_stepp, -1*ang_stepp, 0, + 1*ang_stepp, +2*ang_stepp];

                vel_list = [1:n_vel];
                vel_list(cur_error_list(i).vel) = [];

                plot_buffer = cur_vel_def(vel_list,ang_list);

                % Interp
                fig_corr = figure();
                subplot(1,2,1);
                plot(ang_idx_list, plot_buffer(:,:)', 'Color','#000000','Marker','+');
                hold on;
                plot(ang_idx_list, cur_vel_def(cur_error_list(i).vel, ang_list)', 'Color','#A2142F','Marker','+');

                fit_obj = fit([-2,-1,1,2]', cur_vel_def(cur_error_list(i).vel, ang_list([1,2,4,5]))', 'spline');
                new_vel_def = fit_obj(0);

                plt_corr = plot([0], new_vel_def, 'Color','r','Marker','o','MarkerSize',10);
                plt_rej = plot([0], cur_vel_def(cur_error_list(i).vel, ang_list(3))', 'Color','#A2142F','Marker','*','MarkerSize',10);

                legend([plt_corr;plt_rej],'Suggested correction','Rejected point');
                title("Velocity Deficit, in X");
                xlabel("Relative angel, deg");
                ylabel("red. Velocity Deficit");
                box on;
                grid on;
                hold off;

                % Calc real speed from vel deficit
                vel_corr = -new_vel_def * raw_ref_buffer(cur_error_list(i).vel,1);
                vel_corr = vel_corr + cur_vel_ref(cur_error_list(i).vel,ang_list(3));


                subplot(1,2,2);
                plot_buffer = cur_raw_buffer(vel_list,ang_list);
                plot(ang_idx_list, abs(plot_buffer'), 'Color','#000000','Marker','+');
                hold on;
                plot(ang_idx_list, abs(cur_raw_buffer(cur_error_list(i).vel, ang_list)'), 'Color','#A2142F','Marker','+');

                plt_rej = plot([0],abs(cur_raw_buffer(cur_error_list(i).vel, ang_list(3))'), 'Color','#A2142F','Marker','*','MarkerSize',10);
                plt_corr = plot([0],abs(vel_corr), 'Color','r','Marker','o','MarkerSize',10);
                legend([plt_corr;plt_rej],'Suggested correction','Rejected point');
                hold off;
                title("Velocity in X");
                xlabel("Relative angel, deg");
                ylabel("Velocity, m/s");
                box on;
                grid on;

                dump = questdlg('An irregularity was found, do you want to except the suggested correction?','Correction','Yes','No','Yes');
                if strcmp(dump,'Yes')
                    cur_corr_buffer(cur_error_list(i).vel, ang_list(3)) = vel_corr;
                    n_corrections = n_corrections + cur_error_list(i).length;
                end

            else
                % "Simply" extrapolate from lower velocity

                %Get the changing rates of the vel deficits around the error
                %points
                s_2_index = mod(cur_error_list(i).ang - 2, n_ang);
                s_1_index = mod(cur_error_list(i).ang - 1, n_ang);
                e_1_index = mod(cur_error_list(i).ang + cur_error_list(i).length + 1, n_ang);
                e_2_index = mod(cur_error_list(i).ang + cur_error_list(i).length + 2, n_ang);

                index_buffer = [s_2_index,s_1_index,[cur_error_list(i).ang : cur_error_list(i).ang + cur_error_list(i).length-1], e_1_index, e_2_index];
                index_buffer(find(index_buffer == 0)) = n_ang;


                working_deficits = cur_vel_def(:,index_buffer);


                diff_s2 = working_deficits(cur_error_list(i).vel,1) - working_deficits(cur_error_list(i).vel - 1,1);
        	    diff_s1 = working_deficits(cur_error_list(i).vel,2) - working_deficits(cur_error_list(i).vel - 1,2);
                diff_e1 = working_deficits(cur_error_list(i).vel, end - 1) - working_deficits(cur_error_list(i).vel - 1, end- 1);
                diff_e2 = working_deficits(cur_error_list(i).vel,end) - working_deficits(cur_error_list(i).vel - 1 ,end);

                curve_offset = median([[diff_s2, diff_s1, diff_e1, diff_e2]]);
                plot([1,2,cur_error_list(i).length+2+1, cur_error_list(i).length+2+2],[diff_s2, diff_s1, diff_e1, diff_e2]);

                % corr
                corr_vel_def = zeros(1,cur_error_list(i).length);
                corr_vel = zeros(1,cur_error_list(i).length);
                for j = 1:cur_error_list(i).length
                    corr_vel_def(j) = working_deficits(cur_error_list(i).vel-1,2+j) + curve_offset;
                    corr_vel(j) = corr_vel_def(j) * sensor_obj.raw_ref(cur_error_list(i).vel);
                    corr_vel(j) = -corr_vel(j) + cur_vel_ref(cur_error_list(i).vel, cur_error_list(i).ang+(j - 1));
                    %delta_vel_deficit =  corr_vel_def(j) - working_deficits(cur_error_list(i).vel,2+j);
                    %corr_vel(j) = cur_corr_buffer(cur_error_list(i).vel, index_buffer(j + 2)) - delta_vel_deficit * sensor_obj.raw_ref(cur_error_list(i).vel);
                    % Funktioniert nicht warum auch immer
                end

                % Plot stuff
                subplot(1,2,1);
                plot(working_deficits(:,:)', 'Color','k','Marker','+');
                hold on;
                plot(working_deficits(cur_error_list(i).vel)', 'Color','b','Marker','+');
                for j = 1:cur_error_list(i).length
                    plt_corr = plot([j + 2], corr_vel_def(j), 'Color','r','Marker','o','MarkerSize',10);
                    plt_rej = plot([j + 2], working_deficits(cur_error_list(i).vel, j +2), 'Color','b','Marker','*','MarkerSize',10);
                end
                legend([plt_corr;plt_rej],'Suggested correction','Rejected point');
                title("Velocity Deficit, in X");
                xlabel("Relative index");
                ylabel("red. Velocity Deficit");
                box on;
                grid on;
                hold off;

                subplot(1,2,2);
                plot_buffer = cur_raw_buffer(:,index_buffer);
                plot(abs(plot_buffer'), 'Color','k','Marker','+');
                hold on;
                plot(abs(cur_raw_buffer(cur_error_list(i).vel, index_buffer)'), 'Color','b','Marker','+');
                for j = 1:cur_error_list(i).length
                    plt_corr = plot([j + 2], abs(corr_vel(j)), 'Color','r','Marker','o','MarkerSize',10);
                    plt_rej = plot([j + 2], abs(cur_raw_buffer(cur_error_list(i).vel, index_buffer(j +2))), 'Color','b','Marker','*','MarkerSize',10);
                end
                legend([plt_corr;plt_rej],'Suggested correction','Rejected point');
                title("Velocity, in X");
                xlabel("Relative index");
                ylabel("Velocity, m/s");
                box on;
                grid on;
                hold off;

                dump = questdlg('An irregularity was found, do you want to except the suggested correction?','Correction','Yes','No','Yes');
                if strcmp(dump,'Yes')
                    for j = 1:cur_error_list(i).length
                        cur_corr_buffer(cur_error_list(i).vel, index_buffer(j + 2)) = corr_vel(j);
                        n_corrections = n_corrections + cur_error_list(i).length;
                    end
                end

            end

        end
    end
end
corrected_x = cur_corr_buffer;
end


%% Correct errors in Y
cur_error_list = y_error_list;
if ~isempty(cur_error_list)
if(cur_error_list(1).vel ~= 0)

    cur_error_list = y_error_list;
    cur_vel_def = vel_def_y;
    cur_vel_ref = y_ref;
    cur_raw_buffer = sensor_obj.raw_buffer_y;
    cur_corr_buffer = corrected_y;

    if(cur_error_list(1).vel ~= 0)
        for i = 1:length(cur_error_list)
            if(cur_error_list(i).length == 0)
                % Simply interpolate between ranges left and right
                pp_ang = mod(cur_error_list(i).ang - 2, n_ang);
                p_ang = mod(cur_error_list(i).ang - 1, n_ang);
                ang = mod(cur_error_list(i).ang - 0, n_ang);
                f_ang = mod(cur_error_list(i).ang + 1, n_ang);
                ff_ang = mod(cur_error_list(i).ang + 2, n_ang);
                ang_list = [pp_ang, p_ang, ang, f_ang, ff_ang]; % List of velocities to plot

                ang_stepp = 360 / n_ang;
                ang_idx_list = [-2*ang_stepp, -1*ang_stepp, 0, + 1*ang_stepp, +2*ang_stepp];

                vel_list = [1:n_vel];
                vel_list(cur_error_list(i).vel) = [];

                plot_buffer = cur_vel_def(vel_list,ang_list);

                % Interp
                fig_corr = figure();
                subplot(1,2,1);
                plot(ang_idx_list, plot_buffer(:,:)', 'Color','k','Marker','+');
                hold on;
                plot(ang_idx_list, cur_vel_def(cur_error_list(i).vel, ang_list)', 'Color','b','Marker','+');

                fit_obj = fit([-2,-1,1,2]', cur_vel_def(cur_error_list(i).vel, ang_list([1,2,4,5]))', 'spline');
                new_vel_def = fit_obj(0);

                plt_corr = plot([0], new_vel_def, 'Color','r','Marker','o','MarkerSize',10);
                plt_rej = plot([0], cur_vel_def(cur_error_list(i).vel, ang_list(3))', 'Color','b','Marker','*','MarkerSize',10);

                legend([plt_corr;plt_rej],'Suggested correction','Rejected point');
                title("Velocity Deficit, in Y");
                xlabel("Relative angel, deg");
                ylabel("red. Velocity Deficit");
                box on;
                grid on;
                hold off;

                % Calc real speed from vel deficit
                vel_corr = -new_vel_def * raw_ref_buffer(cur_error_list(i).vel,1);
                vel_corr = vel_corr + cur_vel_ref(cur_error_list(i).vel,ang_list(3));


                subplot(1,2,2);
                plot_buffer = cur_raw_buffer(vel_list,ang_list);
                plot(ang_idx_list, abs(plot_buffer'), 'Color','k','Marker','+');
                hold on;
                plot(ang_idx_list, abs(cur_raw_buffer(cur_error_list(i).vel, ang_list)'), 'Color','b','Marker','+');

                plt_rej = plot([0],abs(cur_raw_buffer(cur_error_list(i).vel, ang_list(3))'), 'Color','b','Marker','*','MarkerSize',10);
                plt_corr = plot([0],abs(vel_corr), 'Color','r','Marker','o','MarkerSize',10);
                legend([plt_corr;plt_rej],'Suggested correction','Rejected point');
                hold off;
                title("Velocity in Y");
                xlabel("Relative angel, deg");
                ylabel("Velocity, m/s");
                box on;
                grid on;

                dump = questdlg('An irregularity was found, do you want to except the suggested correction?','Correction','Yes','No','Yes');
                if strcmp(dump,'Yes')
                    cur_corr_buffer(cur_error_list(i).vel, ang_list(3)) = vel_corr;
                    n_corrections = n_corrections + cur_error_list(i).length;
                end

            else
                % "Simply" extrapolate from lower velocity

                %Get the changing rates of the vel deficits around the error
                %points
                s_2_index = mod(cur_error_list(i).ang - 2, n_ang);
                s_1_index = mod(cur_error_list(i).ang - 1, n_ang);
                e_1_index = mod(cur_error_list(i).ang + cur_error_list(i).length + 1, n_ang);
                e_2_index = mod(cur_error_list(i).ang + cur_error_list(i).length + 2, n_ang);

                index_buffer = [s_2_index,s_1_index,[cur_error_list(i).ang : cur_error_list(i).ang + cur_error_list(i).length-1], e_1_index, e_2_index];
                index_buffer(find(index_buffer == 0)) = n_ang;


                working_deficits = cur_vel_def(:,index_buffer);

                %s_2_poly = fit(sensor_obj.raw_ref(2:end)', working_deficits(:,1),'pchip')
                %s_1_poly = fit(sensor_obj.raw_ref(2:end)', working_deficits(:,2),'pchip')
                %s_2_poly = fit(sensor_obj.raw_ref(2:end)', working_deficits(:,cur_error_list(i).length + 2 + 1),'pchip')
                %s_1_poly = fit(sensor_obj.raw_ref(2:end)', working_deficits(:,cur_error_list(i).length + 2 + 2),'pchip')
                %plot(sensor_obj.raw_ref(2:end), working_deficits(:,1));
                %hold on;
                %plot(sensor_obj.raw_ref(2:end), working_deficits(:,2));
                %plot([5:45], s_2_poly([5:45]));
                %plot([5:45], s_1_poly([5:45]));


                diff_s2 = working_deficits(cur_error_list(i).vel,1) - working_deficits(cur_error_list(i).vel - 1,1);
        	    diff_s1 = working_deficits(cur_error_list(i).vel,2) - working_deficits(cur_error_list(i).vel - 1,2);
                diff_e1 = working_deficits(cur_error_list(i).vel, end - 1) - working_deficits(cur_error_list(i).vel - 1, end- 1);
                diff_e2 = working_deficits(cur_error_list(i).vel,end) - working_deficits(cur_error_list(i).vel - 1 ,end);

                curve_offset = median([[diff_s2, diff_s1, diff_e1, diff_e2]]);
                plot([1,2,cur_error_list(i).length+2+1, cur_error_list(i).length+2+2],[diff_s2, diff_s1, diff_e1, diff_e2]);

                % corr
                corr_vel_def = zeros(1,cur_error_list(i).length);
                corr_vel = zeros(1,cur_error_list(i).length);
                for j = 1:cur_error_list(i).length
                    corr_vel_def(j) = working_deficits(cur_error_list(i).vel -1,2 +j) + curve_offset;
                    corr_vel(j) = -corr_vel_def(j) * sensor_obj.raw_ref(cur_error_list(i).vel);
                    corr_vel(j) = corr_vel(j) + cur_vel_ref(cur_error_list(i).vel, cur_error_list(i).ang  + (j - 1));
                end

                % Plot stuff
                subplot(1,2,1);
                plot(working_deficits(:,:)', 'Color','k','Marker','+');
                hold on;
                plot(working_deficits(cur_error_list(i).vel)', 'Color','b','Marker','+');
                for j = 1:cur_error_list(i).length
                    plt_corr = plot([j + 2], corr_vel_def(j), 'Color','r','Marker','o','MarkerSize',10);
                    plt_rej = plot([j + 2], working_deficits(cur_error_list(i).vel, j +2), 'Color','b','Marker','*','MarkerSize',10);
                end
                legend([plt_corr;plt_rej],'Suggested correction','Rejected point');
                title("Velocity Deficit, in Y");
                xlabel("Relative index");
                ylabel("red. Velocity Deficit");
                box on;
                grid on;
                hold off;

                subplot(1,2,2);
                plot_buffer = cur_raw_buffer(:,index_buffer);
                plot(abs(plot_buffer'), 'Color','k','Marker','+');
                hold on;
                plot(abs(cur_raw_buffer(cur_error_list(i).vel, index_buffer)'), 'Color','b','Marker','+');
                for j = 1:cur_error_list(i).length
                    plt_corr = plot([j + 2], abs(corr_vel(j)), 'Color','r','Marker','o','MarkerSize',10);
                    plt_rej = plot([j + 2], abs(cur_raw_buffer(cur_error_list(i).vel, index_buffer(j +2))), 'Color','b','Marker','*','MarkerSize',10);
                end
                legend([plt_corr;plt_rej],'Suggested correction','Rejected point');
                title("Velocity, in Y");
                xlabel("Relative index");
                ylabel("Velocity, m/s");
                box on;
                grid on;
                hold off;

                dump = questdlg('An irregularity was found, do you want to except the suggested correction?','Correction','Yes','No','Yes');
                if strcmp(dump,'Yes')
                    for j = 1:cur_error_list(i).length
                        cur_corr_buffer(cur_error_list(i).vel, index_buffer(j + 2)) = corr_vel(j);
                        n_corrections = n_corrections + cur_error_list(i).length;
                    end
                end

            end

        end
    end
end
corrected_y = cur_corr_buffer;
end
end


