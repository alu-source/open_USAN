function [lookup_matrix, lookup_error, iter_cost] = Iterative_point_interpolation(sensor_obj)
%ITERATIVE_POINT_INTERPOLATION Summary of this function goes here
%   Detailed explanation goes here

max_iterations = 25;
abort_accure = 0.01;
n_stepps = 181;
v_stepp = 0.45;

n_ang = length(sensor_obj.ang_list);
n_vel = length(sensor_obj.raw_ref);

% Extend the range Velocity range
raw_ref = [0, sensor_obj.raw_ref, sensor_obj.raw_ref(end) * 3]; % sensor_obj.raw_ref(1) * 0.5, sensor_obj.raw_ref
raw_buffer_x = [zeros(1,n_ang);  sensor_obj.raw_buffer_x; sensor_obj.raw_buffer_x(end,:)*3]; % 0.5 * sensor_obj.raw_buffer_x(1,:);
raw_buffer_y = [zeros(1,n_ang);  sensor_obj.raw_buffer_y; sensor_obj.raw_buffer_y(end,:)*3]; % 0.5 * sensor_obj.raw_buffer_y(1,:);
%raw_buffer_x = [zeros(1,n_ang); interp_ref_speeds_x];
%raw_buffer_y = [zeros(1,n_ang); interp_ref_speeds_y];



% Extend angle range
raw_buffer_x = [raw_buffer_x, raw_buffer_x];
raw_buffer_y = [raw_buffer_y, raw_buffer_y];
raw_angle_list = [sensor_obj.ang_list, sensor_obj.ang_list+ 360];
n_raw_angle = length(raw_angle_list);


%fit_obj_x;
%fit_obj_y;

for vi = 2:length(raw_ref(:))
    fit_obj_x(vi-1).fit = fit(raw_angle_list',raw_buffer_x(vi,:)','cubicinterp');
    fit_obj_y(vi-1).fit = fit(raw_angle_list',raw_buffer_y(vi,:)','cubicinterp');
end

raw_angle_list = [-180:0.5:(720-180)];
raw_buffer_x = zeros(length(raw_ref), length(raw_angle_list));
raw_buffer_y = zeros(length(raw_ref), length(raw_angle_list));

n_raw_angle = length(raw_angle_list);

% Build higher fidelity model of the curves
for vi = 2:length(raw_ref(:))
    raw_buffer_x(vi,:) = fit_obj_x(vi-1).fit(raw_angle_list');
    raw_buffer_y(vi,:) = fit_obj_y(vi-1).fit(raw_angle_list');
end



X1 = zeros(1,length(raw_ref)*length(raw_angle_list));
Y1 = zeros(1,length(raw_ref)*length(raw_angle_list));
Z_x = zeros(1,length(raw_ref)*length(raw_angle_list));
Z_y = zeros(1,length(raw_ref)*length(raw_angle_list));


% Fitting a polynom to the surface
for i = 1:length(raw_ref)
    for j = 1:length(raw_angle_list)
        X1((i-1)*n_raw_angle +j) = raw_ref(i);
        Y1((i-1)*n_raw_angle +j) = raw_angle_list(j);
        Z_x((i-1)*n_raw_angle +j) = raw_buffer_x(i,j);
        Z_y((i-1)*n_raw_angle +j) = raw_buffer_y(i,j);
    
    end
end


f_x = fit([X1',Y1'],Z_x','linear');%'cubicinterp');linear
f_y = fit([X1',Y1'],Z_y','linear');%'cubicinterp');linear

% Get error 
fitted_points = f_x([X1',Y1']);
error = abs(diff(Z_x'-fitted_points));
error_idx = find(error > 0.1);

vx_interp_controll = zeros(length(raw_ref),n_raw_angle);
vy_interp_controll = zeros(length(raw_ref),n_raw_angle);

for i = 1:length(raw_ref)
    for j = 1:n_raw_angle
        vx_interp_controll(i,j) = f_x([raw_ref(i),raw_angle_list(j)]);
        vy_interp_controll(i,j) = f_y([raw_ref(i),raw_angle_list(j)]);
    end
end
figure()
surf(vx_interp_controll)
hold on
surf(vy_interp_controll)
hold off

x_t = [0:1:90];
x_n = length(x_t);
y_t = [-180:1:270];
y_n = length(y_t);

X = zeros(1,x_n*y_n);
Y = zeros(1,x_n*y_n);
for ix = 1:x_n
    for iy = 1:y_n
        X((ix-1)*y_n + iy) = x_t(ix);
        Y((ix-1)*y_n + iy) = y_t(iy);
    end
end


lookup_matrix = zeros(n_stepps,n_stepps,2);
lookup_error = zeros(n_stepps,n_stepps);
iter_cost = zeros(n_stepps,n_stepps);

x_val = f_x([X', Y']);
y_val = f_y([X', Y']);



for ix = 1:n_stepps
    %tic
    % Calc Vel and allocate memory
    vx = -(n_stepps - 1)/2 * v_stepp + v_stepp * (ix - 1);
    lookup_inter = zeros(n_stepps,2);
    error_inter = zeros(1,n_stepps);
    iter_inter = zeros(1,n_stepps);
    for iy = 1:n_stepps

        vy = -(n_stepps - 1)/2 * v_stepp + v_stepp * (iy - 1);

        % Finde first minimum
        cost = sqrt((x_val - vx).^2 + (y_val - vy).^2);
        [min_cost,idx] = min(cost);

        %First velocity and angle component
        min_v = X(idx); 
        min_a = Y(idx);

        % Check angle of the guess and correct if needed
        if(min_a < - 160)
            min_a = min_a + 360;
        elseif (min_a > 240)
            min_a = min_a - 360;
        end
        

        %Point list
        point_list_v = [1,1,1,0,0,0,-1,-1,-1,rand(1,9)]';
        point_list_a = [1,0,-1,1,0,-1,1,0,-1,rand(1,9)]';
        current_iter = 1;
        while(current_iter < max_iterations && min_cost > abort_accure)

            % Add some randome points, range from -1.5 to 1.5
            point_list_v(10:18) = (0.5 - rand(1,9)) * 3;
            point_list_v(10:18) = (0.5 - rand(1,9)) * 3;

            %Calc new pointsfor evaluation
            current_iter = current_iter + 1;
            applyed_point_list_v = min_v + point_list_v./(current_iter^1.5);
            applyed_point_list_a = min_a + point_list_a./(current_iter^1.5);
            

            %Interp of points
            x_inter = f_x([applyed_point_list_v, applyed_point_list_a]);
            y_inter = f_y([applyed_point_list_v, applyed_point_list_a]);

            %Finde new minimum
            cost = sqrt((x_inter - vx).^2 +  (y_inter - vy).^2);
            [min_cost, idx] = min(cost);

            %Update index
            min_v = min_v + point_list_v(idx)./(current_iter^1.5);
            min_a = min_a + point_list_a(idx)./(current_iter^1.5);
        end
        
        lookup_inter(iy,1) = -min_v *  sind(min_a);
        lookup_inter(iy,2) = -min_v *  cosd(min_a); % Invertes axis could be fixed here !
        error_inter(iy) = min_cost;
        iter_inter(iy) = current_iter;
    end
    lookup_matrix(ix,:,:) = lookup_inter;
    lookup_error(ix,:) = error_inter;
    iter_cost(ix,:) = iter_inter;
    %toc
end