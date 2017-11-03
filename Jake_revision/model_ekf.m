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
r = 0.25;       % radius of wheels
l = 0.3;        % Length from wheel to robot center
alpha = 2*pi/3; % Angle from wheel to x axis
gdist_mot_lin = 0.01;           % Linear Gaussian Disturbance
gdist_mot_ang = 0.1 *pi/180;    % Angular Gaussian Disturbance
gdist_meas_gps = 0.5;           %GPS Gaussian Disturbance
gdist_meas_gps_cor = 0.01;       %GPS Gaussian Disturbance Corrected
gdist_meas_mag = 10 *pi/180;    % Magnetometor Gaussian Disturbance
if(dist_off)
    gdist_mot_lin = 0;
    gdist_mot_ang = 0;
    gdist_meas_gps = 0;
    gdist_meas_mag = 0;
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


%% Measurement Model
% Setup arrays for measurement plotting
meas_x = zeros(1,cycles);
meas_y = zeros(1,cycles);
meas_theta = zeros(1,cycles);

% Initial location and orientation
meas_x(1) = 0;
meas_y(1) = 0;
meas_theta(1) = 0;


%% Extended Kalman Filter
mu = [1 1 1 1 1 1]'; % mean (mu)
sig = 1*eye(6); % covariance (Sigma)
mu_bar = [1 1 1 1 1 1]';
sig_bar = 1*eye(6);

% Gradient of Motion Model
G = [1, time_step, 0, 0,                              0, 0;
     0, 0, 0, 0, -sin(mu(5))*mot_vx - cos(mu(5))*mot_vy, 0;
     0, 0, 1, time_step,                              0, 0;
     0, 0, 0, 0,  cos(mu(5))*mot_vx - sin(mu(5))*mot_vy, 0;
     0, 0, 0, 0,                              1, time_step;
     0, 0, 0, 0,                              0, 0;
    ];

% Gradient of Measurement Model
H = [1 0 0;
     0 1 0;
     0 0 1;
    ];

% Motion Model Variance
R = [gdist_mot_lin 0 0 0 0 0;   %covariance on x position
     0 0 0 0 0 0;               %no uncertainty on x velocity
     0 0 gdist_mot_lin 0 0 0;   %covariance on y position
     0 0 0 0 0 0;               %no uncertainty on y velocity
     0 0 0 0 gdist_mot_ang 0;   %covariance on angular position
     0 0 0 0 0 0;               %no uncertainty on angular velocity
    ];

% Measurement Model Variance
Q = [gdist_meas_gps 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 gdist_meas_gps 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 gdist_meas_mag 0;
     0 0 0 0 0 0;
    ];

% Measurement Model Variance Corrected GPS
Q_cor = [gdist_meas_gps_cor 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 gdist_meas_gps_cor 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 gdist_meas_mag 0; 
     0 0 0 0 0 0;
    ];

%% Main Loop
figure(1); hold on;
for incr=2:cycles
    % Generate Motion Model
    real_delta_x = mot_vx*time_step*cos(mot_theta(incr-1)) - mot_vy*time_step*sin(mot_theta(incr-1));
    real_delta_y = mot_vx*time_step*sin(mot_theta(incr-1)) + mot_vy*time_step*cos(mot_theta(incr-1));
    
    mot_x(incr) = mot_x(incr-1) + real_delta_x + normrnd(0,gdist_mot_lin);
    mot_y(incr) = mot_y(incr-1) + real_delta_y + normrnd(0,gdist_mot_lin);
    mot_theta(incr) = mot_theta(incr-1) + mot_omega*time_step + normrnd(0,gdist_mot_ang);
    
    scatter(mot_x(incr), mot_y(incr), 'b*');
    
    
    % Generate Measurement Model
    meas_x(incr) = mot_x(incr) + normrnd(0,gdist_meas_gps);
    meas_y(incr) = mot_y(incr) + normrnd(0,gdist_meas_gps);
    meas_theta(incr) = mot_theta(incr) + normrnd(0,gdist_meas_mag) + 9.7 *pi/180;
    
    scatter(meas_x, meas_y, 'r');
    
    
    % Kalman Prediction Update
    G = evaluate_G(mu, [w1, w2, w3], time_step, r);
    mu_bar = evaluate_little_g(mu, [w1, w2, w3], time_step, r, l);
    sig_bar = G*sig*G' + R;
    
    
    % Kalman Measurement Update
    H = evaluate_H(mu, [w1, w2, w3], time_step, r);
    if(mod(cycles, 10) == 0)
        K = sig_bar*H'*inv(H*sig_bar*H' + Q_cor); % Multi-rate Kalman
    else
        K = sig_bar*H'*inv(H*sig_bar*H' + Q);
    end
    y = [meas_x(incr), meas_y(incr), meas_theta(incr)]';
    h = evaluate_little_g(mu_bar, [w1, w2, w3], time_step, r, l);
    mu = mu_bar + K*(y - h([1 3 5]));
    sig = (1*eye(3) - K*H)*sig_bar;
    
    pause(0.05);
end

title('Motion Model for Three Wheel Robot');
xlabel('X Position [m]');
ylabel('Y Position [m]');