clear;clc; close all;
% Different car models, driving around a circle. Types include:
% Dubins - This model uses speed and angular velocity as inputs
% TwoWheel - This model uses left and right wheel speeds as inputs
% Bicycle - This model uses speed and steering angle as inputs, can also
% be used for four wheel Ackermann steered vehicles (like cars)

modeltype = 'Bicycle';
video = false;

%% Create AVI movie object
if video
    vidObj = VideoWriter('car.avi'); % Define filename
    vidObj.Quality = 100; % Set quality
    vidObj.FrameRate = 20; % Set framerate
    open(vidObj); % Open video object
end

%% Simulations
% Time
T = 20; % Duration
dt = 0.1;  % Timestep
tvec = 0:dt:T; % Time vector
n = length(tvec); % Number of timesteps

% Vehicle parameters
r = 1; % Wheel radius
l = 0.3; % Distance from wheel to center

%% Bicycle Model Commands
% TODO 
% [x] length: 30 cm
% [x] gaussian disturbance on x, y position : 0.02 m std dev
% [x] gaussian disturbance on angle theta:  1 degree std dev (0.01744)
% [x] update interval : 0.1 s
% [x] speed: 3 m/s
% [x] steering angle : delta = 10 - t degrees
% [x] steering angle limits : +/- 30 degrees (+/-0.523 rads)
% [x] 20 seconds

% Speed command
v = 3*ones(1,n); % Constant throughout

% Steering Angle
delta = 10*ones(1,n); %in degrees
delta = delta - tvec; % apply steering angle model delta = 10 - t
delta = delta * pi()/180; % convert to radians
for t=1:n
    if(delta(n) > 0.523)
        delta(n) = 0.523;
    end
    if(delta(n) < -0.523)
        delta(n) = -0.523;
    end 
end

%% Body motion integration
x = [0; 0; 0];
disp('Bicycle Car Simulation')
for t=1:n
    x(:,t+1) = bicycle(x(:,t),v(t),delta(t),l,dt);
    noise = [normrnd(0, 0.02) ; normrnd(0, 0.02) ; normrnd(0,0.01744)];
    x(:, t+1) =  x(:, t+1)+ noise;
end
% Create figure of motion
figure(1);
for t=1:2:n % For every second position of the robot
    clf;hold on; % clear the current figure
    
    drawcar(x(1,t),x(2,t),x(3,t),delta(t),0.1,1) % Draw the car
    
    plot(x(1,1:t), x(2,1:t), 'bx'); % Draw the path of the vehicle
    axis equal; % Use same scale on both axes
    axis([-15 10 -25 5]); % Set the dimensions of the plot
    drawnow; % Draw the plot now instead of when Matlab feels like it
    if video 
        writeVideo(vidObj, getframe(gcf)); % Record the plot as a frame in the movie
    end
end

if video
    close(vidObj); % Finish making the movie file
end
