%% Three Wheel Robot Model
% Authors: Jacob Rampertab and Joseph Lundy
% Parameters within the Input Controls section may be altered, all others
% should remain untouched. For Question 5, enable_gps_corrector should be
% turned off (0) and for question 6 is should be turned on (1). 

close all;

%% Input Controls
w1 = -1.5; % Rotation speed of wheel 1
w2 = 2; % Rotation speed of wheel 2
w3 = 1; % Rotation speed of wheel 3
dist_off = 0; % Turn on or off disturbances (Testing only)
enable_gps_corrector = 0; % Enable GPS Corrector (Q6)
makemovie = 1; % Enable video export of EKF

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
if dist_off
    gdist_mot_lin = 0;
    gdist_mot_ang = 0;
    gdist_meas_gps = 0;
    gdist_meas_gps_cor = 0;
    gdist_meas_mag = 0;
end

%% Create AVI object
if(makemovie)
    if(enable_gps_corrector)
        name = 'multi_ekf.avi';
    else
        name = 'ekf.avi';
    end
    vidObj = VideoWriter(name);
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
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
mu = zeros(3,cycles); % mean (mu)
sig = 1*eye(3); % covariance (Sigma)
mu_bar = [1 1 1]';
sig_bar = 1*eye(3);

% Gradient of Motion Model
G = 1*eye(3); % Filled in evaluate_G

% Gradient of Measurement Model
H = 1*eye(3); % Filled in evaluate_H

% Motion Model Variance
R = [gdist_mot_lin 0 0;   %covariance on x position
     0 gdist_mot_lin 0;   %covariance on y position
     0 0 gdist_mot_ang;   %covariance on angular position
    ];

% Measurement Model Variance
Q = [gdist_meas_gps 0 0;
     0 gdist_meas_gps 0;
     0 0 gdist_meas_mag;
    ];

% Measurement Model Variance Corrected GPS
Q_cor = [gdist_meas_gps_cor 0 0;
         0 gdist_meas_gps_cor 0;
         0 0 gdist_meas_mag;
        ];

%% Main Loop
for incr=2:cycles
    % Generate Motion Model
    real_delta_x = mot_vx*time_step*cos(mot_theta(incr-1)) - mot_vy*time_step*sin(mot_theta(incr-1));
    real_delta_y = mot_vx*time_step*sin(mot_theta(incr-1)) + mot_vy*time_step*cos(mot_theta(incr-1));
    
    mot_x(incr) = mot_x(incr-1) + real_delta_x + normrnd(0,gdist_mot_lin);
    mot_y(incr) = mot_y(incr-1) + real_delta_y + normrnd(0,gdist_mot_lin);
    mot_theta(incr) = mot_theta(incr-1) + mot_omega*time_step + normrnd(0,gdist_mot_ang);
    
    
    % Generate Measurement Model
    if mod(incr, 10) == 0 && enable_gps_corrector % Multi-rate Kalman GPS Corrector
        meas_x(incr) = mot_x(incr) + normrnd(0,gdist_meas_gps_cor);
        meas_y(incr) = mot_y(incr) + normrnd(0,gdist_meas_gps_cor);
    else
        meas_x(incr) = mot_x(incr) + normrnd(0,gdist_meas_gps);
        meas_y(incr) = mot_y(incr) + normrnd(0,gdist_meas_gps);
    end
    meas_theta(incr) = mot_theta(incr) + normrnd(0,gdist_meas_mag) + 9.7 *pi/180;
    
    
    % Kalman Prediction Update
    G = evaluate_G(mu(:,incr-1), [w1, w2, w3], time_step, r);
    mu_bar = evaluate_little_g(mu(:,incr-1), [w1, w2, w3], time_step, r, l);
    sig_bar = G*sig*G' + R;
    
    
    % Kalman Measurement Update
    H = evaluate_H(mu(:,incr-1), [w1, w2, w3], time_step, r);
    if mod(incr, 10) == 0 && enable_gps_corrector % Multi-rate Kalman
        K = sig_bar*H'*inv(H*sig_bar*H' + Q_cor);
    else
        K = sig_bar*H'*inv(H*sig_bar*H' + Q);
    end
    y = [meas_x(incr), meas_y(incr), meas_theta(incr)]';
    h = evaluate_little_g(mu_bar, [w1, w2, w3], time_step, r, l);
    mu(:,incr) = mu_bar + K*(y - h);
    sig = (1*eye(3) - K*H)*sig_bar;
    
    
    %Plotting
    figure(1); clf; hold on;
    plot(mot_x, mot_y, 'b*');
    plot(meas_x(incr), meas_y(incr), 'ro');
    plot(mu(1,incr), mu(2,incr), 'gx');
    error_ellipse(sig(1:2,1:2),mu(1:2,incr),0.75);
    error_ellipse(sig(1:2,1:2),mu(1:2,incr),0.95);
    axis equal
    axis([-0.5 1.5 -1 1])
    title('EKF Model for Three Wheel Robot');
    xlabel('X Position [m]');
    ylabel('Y Position [m]');
    legend('True Position','Measured Position','Belief');
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
    
    % Pause to watch plot develop
    %pause(0.1);
end
if (makemovie) close(vidObj); end

% Extra Plotting
figure(2);
sp1 = subplot(3,1,1);
hold(sp1, 'on');
plot((1:cycles), mot_x, 'b*');
plot((1:cycles), mu(1,:), 'gx');
title('Real vs Believed X');
xlabel('Cycle');
ylabel('X Position [m]');
legend('True Position','Belief');
hold(sp1, 'off');

sp2 = subplot(3,1,2);
hold(sp2, 'on');
plot((1:cycles), mot_y, 'b*');
plot((1:cycles), mu(2,:), 'gx');
title('Real vs Believed Y');
xlabel('Cycle');
ylabel('Y Position [m]');
legend('True Position','Belief');
hold(sp2, 'off');

sp3 = subplot(3,1,3);
hold(sp3, 'on');
plot((1:cycles), mot_theta, 'b*');
plot((1:cycles), mu(3,:), 'gx');
title('Real vs Believed Theta');
xlabel('Cycle');
ylabel('Theta Orientation [rad]');
legend('True Orientation','Belief');
hold(sp3, 'off');