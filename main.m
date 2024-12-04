% MATLAB Script: Sphere Following a Path with User-Defined Path Selection
% Clear workspace and figures
clear all; close all; clc;
% Ask user to select path type
disp('Choose the path type:');
disp('1: Circular Path');
disp('2: Straight Horizontal Path');
disp('3: Angled Straight Path');
disp('4: Custom Curved Path with Turns');
path_type = input('Enter 1, 2, 3, or 4 for the corresponding path: ');

animate = input('Do you want to animate the path (Yes or No): ', 's');
% Parameters
% Sphere properties
D_s = 0.006; % Diameter of the sphere (m)
R_s = D_s / 2; % Radius of the sphere (m)
Vol = (4/3) * pi * R_s^3; % Volume of the sphere (m^3)
rho_sphere = 8000; % Density of steel (kg/m^3)
m_sphere = rho_sphere * Vol; % Mass of the sphere (kg)
% Magnetic properties
M = 1.27e6; % Magnetization (A/m)
G_max = 10; % Maximum gradient (T/m)
% Drag properties
Cd = 0.47; % Drag coefficient
rho_fluid = 1025; % Density of fluid (kg/m^3)
A = pi * R_s^2; % Reference area (m^2)
V_blood = 0.001; % Velocity of blood (m/s)
% Path selection and properties
N_p = 500; % Number of points along the path
if path_type == 1
   % Circular path
   disp('You selected Circular Path.');
   R_path = 0.05; % Radius of the circular path (m)
   theta_p = linspace(0, pi, N_p); % Parameter for path from right to left
   x_p = R_path * cos(theta_p); % x-coordinates of the circular path
   y_p = R_path * sin(theta_p); % y-coordinates of the circular path
elseif path_type == 2
   % Straight path
   disp('You selected Straight Horizontal Path.');
   R_path = 0.05; % Length of the straight path along x-axis (m)
   x_p = linspace(0, R_path, N_p); % x-coordinates for the straight path
   y_p = zeros(1, N_p); % y-coordinates are all zero for a straight path
elseif path_type == 3
   % Angled straight path
   disp('You selected Angled Straight Path.');
   R_path = 0.05; % Length of the straight path along x-axis (m)
   angle_deg = 45; % Angle of the path in degrees
   angle_rad = deg2rad(angle_deg); % Convert to radians
   x_p = linspace(0, cos(angle_rad)* R_path, N_p); % x-coordinates for the straight path
   y_p = linspace(0, sin(angle_rad)* R_path, N_p); % x-coordinates for the straight path

elseif path_type == 4
   % Custom curved path with turns (e.g., sine wave)
   disp('You selected Custom Curved Path with Turns.');
   R_path = 0.05; % Radius of the circular path (m)
   theta_p = linspace(0, pi, N_p); % Parameter for path from right to left
   x_p = R_path * cos(theta_p); % x-coordinates of the circular path
   y_p = R_path * 0.5 * sin(5 * pi * x_p / 0.1); % Sine wave with 2.5 periods; % y-coordinates of the circular path

else  
    error('Invalid input! Please run the program again and enter 1, 2, 3, or 4.');
end




P_path = [x_p', y_p']; % Path coordinates
% Vessel properties
clearance = 0.001; % Clearance (m)
R_vessel = R_s + clearance; % Vessel wall thickness (m)
R_inner = R_path - R_vessel; % Inner boundary radius
R_outer = R_path + R_vessel; % Outer boundary radius
% Planned velocity profile
V0 = 0.01; % Maximum desired velocity (m/s)
V_p = V0 * ones(1, N_p); % Uniform velocity profile (m/s)
% Time parameters
dt = 0.01; % Time step (s)
t_end = 2 * pi * R_path / V0; % Total simulation time (s)
time = 0:dt:t_end; % Time vector
% PID Controller parameters
Kp = 1; % Proportional gain
Ki = 0; % Integral gain
Kd = 0.3; % Derivative gain
k_weight = 0.3; % Coefficient k for position error weighting
% Kalman Filter parameters
% State vector: x = [position_x; position_y; velocity_x; velocity_y]
n_states = 4; % Number of states
n_measurements = 2; % Number of measurements (position_x, position_y)
% Process noise covariance
Q = 1e-4 * eye(n_states);
% Measurement noise covariance
R = 1e-3 * eye(n_measurements);
% Initial state estimate
x_est = [x_p(1); y_p(1); 0; 0];
% Initial estimation covariance matrix
P_est = 1e-1 * eye(n_states);
% State transition matrix (Assuming constant velocity model)
A_kf = [1 0 dt 0;
        0 1 0 dt;
        0 0 1 0;
        0 0 0 1];
% Measurement matrix
H_kf = [1 0 0 0;
        0 1 0 0];
% Initialize variables
Ps_true = [x_p(1); y_p(1)]; % True initial position of the sphere (m)
Vs_true = [0.001; 0]; % True initial velocity (m/s), small initialpush
error_int = [0; 0]; % Integral of error
prev_error = [0; 0]; % Previous total error
% Measurement noise standard deviation
position_noise_std = 1e-3; % Standard deviation of position measurements (m)
% Arrays to store simulation data
Ps_history = zeros(2, length(time)); % True position history
Vs_history = zeros(2, length(time)); % True velocity history
Ps_est_history = zeros(2, length(time)); % Estimated position history
Vs_est_history = zeros(2, length(time)); % Estimated velocity history
G_history = zeros(1, length(time)); % Gradient history
V_error = zeros(2, length(time)); % Velocity error history
P_error = zeros(2, length(time)); % Position error history

% Initialize target index
target_idx = 1; % Index of the current target point along the path

% Define tolerance for reaching the target point
tolerance = 0.001; % 1 mm tolerance
% Define maximum allowed deviation from the path
max_deviation = R_s; % Maximum allowed deviation (robot radius)
% Simulation loop
for i = 1:length(time)
 % Current time
 t = time(i);
 % Simulate measurement noise in position
 measurement_noise = position_noise_std * randn(2, 1);
 z_measured = Ps_true + measurement_noise; % Noisy position measurement
 % Kalman Filter Prediction Step
 x_pred = A_kf * x_est;
 P_pred = A_kf * P_est * A_kf' + Q;
 % Kalman Filter Update Step
 y_kf = z_measured - H_kf * x_pred; % Innovation
 S_kf = H_kf * P_pred * H_kf' + R; % Innovation covariance
 K_kf = P_pred * H_kf' / S_kf; % Kalman gain
 x_est = x_pred + K_kf * y_kf; % Updated state estimate
 P_est = (eye(n_states) - K_kf * H_kf) * P_pred; % Updated estimation covariance
 % Extract estimated position and velocity
 Ps_est = x_est(1:2); % Estimated position
 Vs_est = x_est(3:4); % Estimated velocity
 % Check if the robot is close enough to the current target point
 distance_to_target = norm(Ps_est - P_path(target_idx, :)');
 if distance_to_target < tolerance && target_idx < N_p
     % Move to the next target point
     target_idx = target_idx + 1;
 end
 % Set the target point
 Pc = P_path(target_idx, :)'; % Target point on the path
 Vc = V_p(target_idx); % Desired speed at Pc
 % Compute position error
 position_error = Pc - Ps_est; % Position error vector (m)
 % Compute direction unit vector safely
 distance = norm(position_error);
 if distance ~= 0
    direction_unit = position_error / distance;
 else

    direction_unit = [0; 0]; % No direction when at the target point
 end
 % Desired velocity vector
 Vc_vector = Vc * direction_unit;
 % Compute velocity error
 velocity_error = Vc_vector - Vs_est; % Velocity error vector (m/s)
 % Total error for PID (weighted sum of position error and velocity error)
 total_error = k_weight * position_error + velocity_error;
 % PID Controller
 error_int = error_int + total_error * dt; % Integral term
 derivative_error = (total_error - prev_error) / dt; % Derivative term
 PID_output = Kp * total_error + Ki * error_int + Kd * derivative_error;
% PID output
 prev_error = total_error; % Update previous error
 % Limit the PID output to prevent excessive acceleration
 PID_output_mag = norm(PID_output);
 a_max = 100; % Maximum allowable acceleration (m/s^2)
 if PID_output_mag > a_max
    PID_output = (PID_output / PID_output_mag) * a_max;
 end
 % Check if the robot is within the allowed deviation from the path
 distance_from_center = norm(Ps_est - [0; 0]);
 if abs(distance_from_center - R_path) > max_deviation
     % The robot is outside the acceptable path area
     % Adjust control input to bring it back
     % Increase k_weight to prioritize position correction
     total_error = 10 * position_error + velocity_error;
     PID_output = Kp * total_error + Ki * error_int + Kd * derivative_error;
 end
 % Desired acceleration
 a_desired = PID_output; % Desired acceleration vector (m/s^2)
 % Compute the net force required (F_net = m * a_desired)
 F_net = m_sphere * a_desired;
 % Drag force
 V_rel = Vs_true - V_blood * direction_unit; % Relative velocity (m/s)
 V_rel_norm = norm(V_rel);
 if V_rel_norm ~= 0
    F_drag = 0.5 * Cd * rho_fluid * A * V_rel_norm^2 * (-V_rel / V_rel_norm); % Drag force (N)

 else
    F_drag = [0; 0];
 end
 % Optimal control force (F_opt)
 %F_opt = 0.5 * Cd * rho_fluid * A * norm(V_blood - Vc_vector)^2 * (-direction_unit);
 F_opt = 0 ;
 % Magnetic force required (Fs = F_net - F_drag)
 Fs_required = F_net + F_opt - F_drag;
 % Compute the gradient G required (Fs = G * M * Vol * Fs_unit)
 Fs_norm = norm(Fs_required);
 if Fs_norm ~= 0
     Fs_unit = Fs_required / Fs_norm;
     G_required = Fs_norm / (M * Vol);
 else
     Fs_unit = [0; 0];
     G_required = 0;
 end
 % Limit G to G_max
 if G_required > G_max
     G = G_max;
     Fs_applied_mag = G * M * Vol;
     Fs_applied = Fs_applied_mag * Fs_unit;
 else
     G = G_required;
     Fs_applied = Fs_required;
 end
 G_history(i) = G; % Store gradient history
 % Update true acceleration, velocity, and position
 a_s_true = (Fs_applied + F_drag) / m_sphere; % True acceleration (m/s^2)
 Vs_true = Vs_true + a_s_true * dt; % True velocity (m/s)
 Ps_true = Ps_true + Vs_true * dt; % True position (m)
 % Store data
 Ps_history(:, i) = Ps_true;
 Vs_history(:, i) = Vs_true;
 Ps_est_history(:, i) = Ps_est;
 Vs_est_history(:, i) = Vs_est;
 V_error(:, i) = velocity_error;
 P_error(:, i) = position_error;
 % Check for NaN values and stop simulation if they occur
 if any(isnan(Ps_true)) || any(isnan(Vs_true)) || any(isnan(Ps_est)) || any(isnan(Vs_est))
     disp('Simulation stopped due to NaN values.');
     Ps_history = Ps_history(:, 1:i);
     Vs_history = Vs_history(:, 1:i);
     Ps_est_history = Ps_est_history(:, 1:i);
     Vs_est_history = Vs_est_history(:, 1:i);
     V_error = V_error(:, 1:i);
     P_error = P_error(:, 1:i);
    
     G_history = G_history(1:i);
     time = time(1:i);
     break;
 end
end

if animate == "No"
    % % Plotting the results
    figure;
    hold on;
    % Select visualization based on path type
    if (path_type == 1 )
       % Circular path visualization with vessel boundaries
       theta_full = linspace(0, pi, 1000);
       x_inner = R_inner * cos(theta_full);
       y_inner = R_inner * sin(theta_full);
       x_outer = R_outer * cos(theta_full);
       y_outer = R_outer * sin(theta_full);
       % Fill the vessel area
       fill([x_outer, fliplr(x_inner)], [y_outer, fliplr(y_inner)], [0.9, 0.9, 0.9], 'EdgeColor', 'none');
       title('Sphere Following a 2D Circular Path with Vessel Boundaries');
    elseif (path_type == 2 )
       % Straight path visualization with vessel boundaries
       x_inner = linspace(0, R_path, 1000);
       y_inner = -R_vessel * ones(1, 1000); % Lower vessel boundary
       x_outer = linspace(0, R_path, 1000);
       y_outer = R_vessel * ones(1, 1000); % Upper vessel boundary
       % Fill the vessel area
       fill([x_outer, fliplr(x_inner)], [y_outer, fliplr(y_inner)], [0.9, 0.9, 0.9], 'EdgeColor', 'none');
       title('Sphere Following a 2D Straight Path with Vessel Boundaries');
    elseif path_type == 3
       % Angled straight path visualization with vessel boundaries
       disp('Visualizing vessel boundaries for Angled Straight Path.');
       % Calculate the unit normal vector to the path
       dx = x_p(end) - x_p(1);
       dy = y_p(end) - y_p(1);
       path_length = sqrt(dx^2 + dy^2);
       tangent = [dx; dy] / path_length;
       normal = [-tangent(2); tangent(1)]; % Rotate tangent by 90 degrees to get normal
    
       % Offset the path to get vessel boundaries
       x_inner = x_p - R_vessel * normal(1);
       y_inner = y_p - R_vessel * normal(2);
       x_outer = x_p + R_vessel * normal(1);
       y_outer = y_p + R_vessel * normal(2);
    
       % Fill the vessel area
       fill([x_outer, fliplr(x_inner)], [y_outer, fliplr(y_inner)], [0.9, 0.9, 0.9], 'EdgeColor', 'none');
       title('Sphere Following an Angled Straight Path with Vessel Boundaries');
    elseif path_type == 4
       % Custom curved path visualization with vessel boundaries
       disp('Visualizing vessel boundaries for Custom Curved Path.');
       % Calculate the normal vectors at each point
       % Compute numerical derivatives
       dx = gradient(x_p);
       dy = gradient(y_p);
       % Compute the tangent vectors
       tangent = [dx; dy];
       tangent_norm = sqrt(dx.^2 + dy.^2);
       % Avoid division by zero
       tangent_norm(tangent_norm == 0) = eps;
       % Normalize the tangent vectors
       tangent_unit = [dx ./ tangent_norm; dy ./ tangent_norm];
       % Compute normal vectors by rotating tangent vectors by 90 degrees
       normal = [-tangent_unit(2, :); tangent_unit(1, :)];
    
       % Offset the path to get vessel boundaries
       x_inner = x_p - R_vessel * normal(1, :);
       y_inner = y_p - R_vessel * normal(2, :);
       x_outer = x_p + R_vessel * normal(1, :);
       y_outer = y_p + R_vessel * normal(2, :);
    
       % Fill the vessel area
       fill([x_outer, fliplr(x_inner)], [y_outer, fliplr(y_inner)], [0.9, 0.9, 0.9], 'EdgeColor', 'none');
       title('Sphere Following a Custom Curved Path with Vessel Boundaries');
    end
    % Plot the planned path
    plot(x_p, y_p, 'k--', 'LineWidth', 1.5);
    % Plot the true sphere's trajectory
    plot(Ps_history(1, :), Ps_history(2, :), 'b', 'LineWidth', 2);
    % Plot the estimated sphere's trajectory
    plot(Ps_est_history(1, :), Ps_est_history(2, :), 'r', 'LineWidth', 1);
    % Plot the starting and ending points
    plot(x_p(1), y_p(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(x_p(end), y_p(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    % Labels and legend
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend('Vessel Boundaries', 'Planned Path', 'True Trajectory', 'Estimated Trajectory', 'Start Point', 'End Point');
    axis equal;
    grid on;
    hold off;
    % Plot Gradient History
    figure;
    plot(time, G_history, 'r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Gradient G (T/m)');
    title('Gradient Applied over Time');
    grid on;

    % Compute the magnitude of position and velocity errors
    position_error_mag = sqrt(P_error(1, :).^2 + P_error(2, :).^2);
    velocity_error_mag = sqrt(V_error(1, :).^2 + V_error(2, :).^2);
    
    % Plot Position Error over Time
    figure;
    plot(time, position_error_mag, 'b', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position Error Magnitude (m)');
    title('Position Error over Time');
    grid on;
    
    % Plot Velocity Error over Time
    figure;
    plot(time, velocity_error_mag, 'g', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity Error Magnitude (m/s)');
    title('Velocity Error over Time');
    grid on;
elseif animate == "Yes"
    % Plotting the results with animation
    figure;
    hold on;
    
    % Select visualization based on path type
    if (path_type == 1 )
       % Circular path visualization with vessel boundaries
       theta_full = linspace(0, pi, 1000);
       x_inner = R_inner * cos(theta_full);
       y_inner = R_inner * sin(theta_full);
       x_outer = R_outer * cos(theta_full);
       y_outer = R_outer * sin(theta_full);
       % Fill the vessel area
       fill([x_outer, fliplr(x_inner)], [y_outer, fliplr(y_inner)], [0.9, 0.9, 0.9], 'EdgeColor', 'none');
       title('Sphere Following a 2D Circular Path with Vessel Boundaries');
    elseif (path_type == 2 )
       % Straight path visualization with vessel boundaries
       x_inner = linspace(0, R_path, 1000);
       y_inner = -R_vessel * ones(1, 1000); % Lower vessel boundary
       x_outer = linspace(0, R_path, 1000);
       y_outer = R_vessel * ones(1, 1000); % Upper vessel boundary
       % Fill the vessel area
       fill([x_outer, fliplr(x_inner)], [y_outer, fliplr(y_inner)], [0.9, 0.9, 0.9], 'EdgeColor', 'none');
       title('Sphere Following a 2D Straight Path with Vessel Boundaries');
    elseif path_type == 3
       % Angled straight path visualization with vessel boundaries
       disp('Visualizing vessel boundaries for Angled Straight Path.');
       % Calculate the unit normal vector to the path
       dx = x_p(end) - x_p(1);
       dy = y_p(end) - y_p(1);
       path_length = sqrt(dx^2 + dy^2);
       tangent = [dx; dy] / path_length;
       normal = [-tangent(2); tangent(1)]; % Rotate tangent by 90 degrees to get normal
    
       % Offset the path to get vessel boundaries
       x_inner = x_p - R_vessel * normal(1);
       y_inner = y_p - R_vessel * normal(2);
       x_outer = x_p + R_vessel * normal(1);
       y_outer = y_p + R_vessel * normal(2);
    
       % Fill the vessel area
       fill([x_outer, fliplr(x_inner)], [y_outer, fliplr(y_inner)], [0.9, 0.9, 0.9], 'EdgeColor', 'none');
       title('Sphere Following an Angled Straight Path with Vessel Boundaries');
    elseif path_type == 4
       % Custom curved path visualization with vessel boundaries
       disp('Visualizing vessel boundaries for Custom Curved Path.');
       % Calculate the normal vectors at each point
       % Compute numerical derivatives
       dx = gradient(x_p);
       dy = gradient(y_p);
       % Compute the tangent vectors
       tangent = [dx; dy];
       tangent_norm = sqrt(dx.^2 + dy.^2);
       % Avoid division by zero
       tangent_norm(tangent_norm == 0) = eps;
       % Normalize the tangent vectors
       tangent_unit = [dx ./ tangent_norm; dy ./ tangent_norm];
       % Compute normal vectors by rotating tangent vectors by 90 degrees
       normal = [-tangent_unit(2, :); tangent_unit(1, :)];
    
       % Offset the path to get vessel boundaries
       x_inner = x_p - R_vessel * normal(1, :);
       y_inner = y_p - R_vessel * normal(2, :);
       x_outer = x_p + R_vessel * normal(1, :);
       y_outer = y_p + R_vessel * normal(2, :);
    
       % Fill the vessel area
       fill([x_outer, fliplr(x_inner)], [y_outer, fliplr(y_inner)], [0.9, 0.9, 0.9], 'EdgeColor', 'none');
       title('Sphere Following a Custom Curved Path with Vessel Boundaries');
    end
    
    % Plot the planned path
    plot(x_p, y_p, 'k--', 'LineWidth', 1.5);
    
    % Plot the starting and ending points
    plot(x_p(1), y_p(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(x_p(end), y_p(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % Labels and legend
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    axis equal;
    grid on;
    
    % Initialize plot handles for the trajectories
    true_traj_plot = plot(NaN, NaN, 'b', 'LineWidth', 2); % True trajectory
    est_traj_plot = plot(NaN, NaN, 'r', 'LineWidth', 1); % Estimated trajectory
    
    % Add markers for current positions
    true_marker = plot(NaN, NaN, 'bo', 'MarkerFaceColor', 'b');
    est_marker = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r');
    
    % Set axis limits for better visualization
    xlim([min([x_p, x_outer, x_inner]) - 0.01, max([x_p, x_outer, x_inner]) + 0.01]);
    ylim([min([y_p, y_outer, y_inner]) - 0.01, max([y_p, y_outer, y_inner]) + 0.01]);
    legend('Vessel Boundaries', 'Planned Path', 'Start Point', 'End Point', 'Actual Trajectory', 'Estimated Trajectory');

    % Animation loop
    for i = 1:length(time)
        % Update true trajectory
        set(true_traj_plot, 'XData', Ps_history(1, 1:i), 'YData', Ps_history(2, 1:i));
        % Update estimated trajectory
        set(est_traj_plot, 'XData', Ps_est_history(1, 1:i), 'YData', Ps_est_history(2, 1:i));
        % Update current positions
        set(true_marker, 'XData', Ps_history(1, i), 'YData', Ps_history(2, i));
        set(est_marker, 'XData', Ps_est_history(1, i), 'YData', Ps_est_history(2, i));
        % Refresh the plot
        drawnow;
        % Optionally, pause to control the speed of the animation
        % pause(0.001);
    end
    
    % After the animation loop, adjust the legend to include trajectories
    legend('Vessel Boundaries', 'Planned Path', 'Start Point', 'End Point', 'Actual Trajectory', 'Estimated Trajectory');
    
    % Plot Gradient History
    figure;
    plot(time, G_history, 'r', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Gradient G (T/m)');
    title('Gradient Applied over Time');
    grid on;

    % Compute the magnitude of position and velocity errors
    position_error_mag = sqrt(P_error(1, :).^2 + P_error(2, :).^2);
    velocity_error_mag = sqrt(V_error(1, :).^2 + V_error(2, :).^2);
    
    % Plot Position Error over Time
    figure;
    plot(time, position_error_mag, 'b', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Position Error Magnitude (m)');
    title('Position Error over Time');
    grid on;
    
    % Plot Velocity Error over Time
    figure;
    plot(time, velocity_error_mag, 'g', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Velocity Error Magnitude (m/s)');
    title('Velocity Error over Time');
    grid on;
end

