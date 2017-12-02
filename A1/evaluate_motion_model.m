function current_state = evaluate_motion_model(Prev_state, Omega,r, l, T, sigma_theta, sigma_xy)
%% Three wheel robot model kinematics 
%  prev_state_vec = [x, x_dot, y, y_dot, theta, theta_dot]', 
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
% calculate robot's velocity in its relative frame
xdot_local = (v3-v2)*0.866; %cos(30) ~= 0.866
ydot_local = v1 - (v3+v2)*0.5; %cos(60) = 0.5
% calculate robot's velocity in its global frame
xdot_global = cos(theta) * xdot_local - sin(theta)*ydot_local;
ydot_global = sin(theta) * xdot_local + cos(theta)*xdot_local;

%% Update x-y position of Robot
% Gaussian Noise for positional disturbance
x_disturbance = normrnd(0,sigma_xy);
y_disturbance = normrnd(0,sigma_xy);
% updating x-y with gaussian noise
x = Prev_state(1); y = Prev_state(3);
x_new = x + xdot_global * T + x_disturbance;
y_new = y + ydot_global * T + y_disturbance;
%% Return Updated State Vector
current_state = [x_new,xdot_global,y_new,ydot_global, theta_new, thetadot];