function current_state = evaluate_motion_model(Prev_state, Omega,r, l, T, sigma_theta, sigma_xy)
%% Three wheel robot model kinematics 
%  prev_state_vec = [x, y, x_dot, y_dot, theta, theta_dot]', 
%  inputs = wheel speeds[wheel_1, wheel_2, wheel_3] 
%  constants: r (wheel radius), l (robot radius), T (time step)
%% Update Angular Velocity of Robot
% previous angle
theta = Prev_state(5);
% angular velocity of each wheel 
w1 = Omega(1); w2 = Omega(2); w3 = Omega(3);
% update theta_dot
thetadot = r/l*(w1 + w2 + w3);
%% Update Angular Position of Robot
% Gaussian Noise for angular disturbance
angular_disturbance = normrnd(0,sigma_theta);
% Update angle with guassian noise
theta_new = theta + thetadot * T + angular_disturbance;

%% Update x-y velocity of Robot
% velocities at 3 wheel hubs
v1 = r * w1; v2 = r * w2; v3 = r * w3;
% absolute angular position of each wheel
offsetAngle = 2*pi/3;
a1 = theta; a2 = a1 + offsetAngle; a3 = a2 + offsetAngle;
% summing x and y velocity components, with gaussian noise
xdot = v1*cos(a1) + v2*cos(a2) + v3*cos(a3);
ydot = v1*sin(a1) + v2*sin(a2) + v3*sin(a3);

%% Update x-y position of Robot
% Gaussian Noise for positional disturbance
x_disturbance = normrnd(0,sigma_xy);
y_disturbance = normrnd(0,sigma_xy);
% updating x-y with gaussian noise
x = Prev_state(1); y = Prev_state(2);
x_new = x + xdot * T + x_disturbance;
y_new = y + ydot * T + y_disturbance;
%% Return Updated State Vector
current_state = [x_new,y_new,xdot,ydot, theta_new, thetadot];