function [lookup_matrix, lookup_error] = Calculate_correction(sensor_obj)
%CALCULATE_CORRECTION Summary of this function goes here
%   Detailed explanation goes here

%% Correction
% Convert angels to positiv values
for i = 1: length(sensor_obj.ang_list)
    if sensor_obj.ang_list(i) < 0
        sensor_obj.ang_list(i) = 360 + sensor_obj.ang_list(i);
    end
end


% Calculate stuff for angle correction
angles_ideal = deg2rad(sensor_obj.ang_list - 360*[sensor_obj.ang_list>180]);
% Check zero for zero !!!
angles_raw = atan2(-sensor_obj.raw_buffer_y, -sensor_obj.raw_buffer_x); 
angles_raw = unwrap(angles_raw, [], 2);
angles_raw_mean = mean(angles_raw, 1);
angles_raw_mean = unwrap(angles_raw_mean) - pi/2;

% Calculate stuff for speed correction
velocities_ideal = sensor_obj.raw_ref';
velocities_raw = ((sensor_obj.raw_buffer_y.^2) + (sensor_obj.raw_buffer_x.^2)).^0.5;
velocities_raw_norm = velocities_raw ./ (velocities_ideal*ones(1, length(sensor_obj.ang_list)));
velocities_raw_norm_mean = mean(velocities_raw_norm, 1);
velocities_raw_mean_angle = mean(velocities_raw, 2);

% Extending for 
angles_raw_mean = [angles_raw_mean(:, end)-2*pi, angles_raw_mean, angles_raw_mean(:, 1)+2*pi];
velocities_raw_mean_angle;
angles_ideal = [angles_ideal(:, end)-2*pi, angles_ideal, angles_ideal(:, 1)+2*pi];
velocity_gain = 1./velocities_raw_norm;
velocity_gain = [velocity_gain(:, end), velocity_gain, velocity_gain(:, 1)];

%% Eval the correction
figure()
grid on
hold on
box on
axis equal

errors = [];

for i = 1:length(sensor_obj.raw_ref)-1
    for j = 1:length(sensor_obj.ang_list)
        
        x_ideal = -velocities_ideal(i)*sin(angles_ideal(j+1));
        y_ideal = -velocities_ideal(i)*cos(angles_ideal(j+1));
        
        plot(x_ideal, y_ideal, 'k+')
        
        angle_raw = atan2(-sensor_obj.raw_buffer_y(i, j), - sensor_obj.raw_buffer_x(i, j)) -pi/2;
        if angle_raw > pi
            angle_raw = angle_raw - pi*2;
        elseif angle_raw < -pi
            angle_raw = angle_raw + pi*2;
        end
        
        velocity_raw = sqrt(sensor_obj.raw_buffer_x(i, j)^2 + sensor_obj.raw_buffer_y(i, j)^2);

        angle_calib = interp1(angles_raw_mean, angles_ideal, angle_raw, 'linear', 'extrap');

        if velocity_raw > 1.6
            gain = interp2(angles_raw_mean, velocities_raw_mean_angle, velocity_gain, angle_raw, velocity_raw, 'spline');
            x_calibrated = -velocity_raw * gain * sin(angle_calib);
            y_calibrated = -velocity_raw * gain * cos(angle_calib);
        elseif velocity_raw > 0.5
            velocity_raw = velocity_raw * 3.5;
            gain = interp2(angles_raw_mean, velocities_raw_mean_angle, velocity_gain, angle_raw, velocity_raw, 'linear');
            x_calibrated = -velocity_raw/3.5 * gain * sin(angle_calib);
            y_calibrated = -velocity_raw/3.5 * gain * cos(angle_calib);
        else
            x_calibrated = -sensor_obj.raw_buffer_x(i, j);
            y_calibrated = sensor_obj.raw_buffer_y(i, j);
        end

        

        
        plot([x_ideal, x_calibrated], [y_ideal, y_calibrated], 'b')
        plot(x_calibrated, y_calibrated, 'r+')
        
        errors(end+1) = sqrt((x_ideal-x_calibrated)^2 + (y_ideal-y_calibrated)^2);

        if isnan(x_calibrated) || isnan(y_calibrated)
            pause(1);
        end    
        
        
    end
end

xlabel('x-Geschwindigkeit, m/s')
ylabel('y-Geschwindigkeit, m/s')
legend({'Ideal', 'Delta', 'Kalibriert', })

figure()
histogram(errors)
xlabel('Abs.-Geschwindigkeitsfehler, m/s')
ylabel('Häufigkeit')

mean_error = mean(errors)
max_error = max(errors)



%% Saving nessary data to struct


end

