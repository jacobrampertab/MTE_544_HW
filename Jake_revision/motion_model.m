%% Three Wheel Robot Model
% Description TBD

close all;

%% Input Controls
w1 = -1.5; % Rotation speed of wheel 1
w2 = 2; % Rotation speed of wheel 2
w3 = 1; % Rotation speed of wheel 3
dist_off = 0; % Turn on or off disturbances

% Forward:  w1 = 0,     w2 = -1,    w3 = 1
% Rotate:   w1 = 1,     w2 = 1,     w3 = 1
% Spiral:   w1 = 1,     w2 = -1.5   w3 = 1.5

% Time Parameters
runtime = 15;   % seconds for simulation runtime
time_step = 0.1;    % seconds for evaluation time step

%% Constants
% Time Parameters
cycles = round(runtime/time_step);  % N number of steps total to evaluate

% Physical Robot Parameters
r = 0.25;   % radius of wheels
l = 0.3;    % Length from wheel to robot center
alpha = 2*pi/3; % Angle from wheel to x axis
gau_dist_lin = 0.01;    % Linear Gaussian Disturbance
gau_dist_ang = 0.1*pi/180;  % Angular Gaussian Disturbance
if(dist_off)
    gau_dist_lin = 0;
    gau_dist_ang = 0;
end

%% Motion Model
% Motion model for change in x, y, omega in local frame
mot_vx = (w3 - w2)*r*cos(alpha - pi/2);
mot_vy = r*w1 - (w3 + w2)*r*cos(pi - alpha);
mot_omega = r / l * (w1 + w2 + w3);

% Cleaning up values
mot_vx = round(mot_vx, 3);
mot_vy = round(mot_vy, 3);
mot_omega = round(mot_omega, 3);

% Setup arrays for motion plotting
mot_x = zeros(1,cycles);
mot_y = zeros(1,cycles);
mot_theta = zeros(1,cycles);

% Initial location and orientation
mot_x(1) = 0;
mot_y(1) = 0;
mot_theta(1) = 0;
figure(1);

for incr=2:cycles
    real_delta_x = mot_vx*time_step*cos(mot_theta(incr-1)) - mot_vy*time_step*sin(mot_theta(incr-1));
    real_delta_y = mot_vx*time_step*sin(mot_theta(incr-1)) + mot_vy*time_step*cos(mot_theta(incr-1));
    
    mot_x(incr) = mot_x(incr-1) + real_delta_x + normrnd(0,gau_dist_lin);
    mot_y(incr) = mot_y(incr-1) + real_delta_y + normrnd(0,gau_dist_lin);
    mot_theta(incr) = mot_theta(incr-1) + mot_omega*time_step + normrnd(0,gau_dist_ang);
    
    plot(mot_x, mot_y, 'b*');
    pause(0.05);
end

title('Motion Model for Three Wheel Robot');
xlabel('X Position [m]');
ylabel('Y Position [m]');