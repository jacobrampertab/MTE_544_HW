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
l = 0.3; % Distance from wheel to center

% Controller Parameters
k = 5; % steering correction gain

% carrot controller
r = 1; % track one meter ahead

%% Bicycle Model Commands

% Speed command
v = 3*ones(1,n); % Constant throughout

% Steering Angle
delta = zeros(1,n); %in rad

% Heading errors
dHeadErr = 0;
curHeadErr = 0;
prevHeadErr = 0;

%% Waypoints
waypoints = [20 0; 20 5; 0 5; 0 0];
numWayPoints = length(waypoints);
prevWayPoint = 4;
curWayPoint = 1;

%% Body motion integration
x = zeros(3,n);

disp('Bicycle Car Simulation')
for t=1:n-1
    % TODO use r here
    [crosstrack_error outside] = distanceToLineSegment(waypoints(prevWayPoint,:), waypoints(curWayPoint,:), x(1:2,t)');
    if(outside)
        disp("reached point");
        prevWayPoint = curWayPoint;
        curWayPoint = curWayPoint + 1;
        if(curWayPoint > numWayPoints)
            curWayPoint = 1;
        end
        [crosstrack_error outside] = distanceToLineSegment(waypoints(prevWayPoint,:), waypoints(curWayPoint,:), x(1:2,t)');
        dHeadErr = 0;
        curHeadErr = -pi()/2;
        prevHeadErr = 0;
        delta(t+1) = -pi()/2;
    else
        dHeadErr = -v(t+1)*sin(delta(t))/l;
        prevHeadErr = curHeadErr;
        curHeadErr = prevHeadErr + dHeadErr*dt;
        delta(t+1) = curHeadErr + atan(k*crosstrack_error/v(t+1));
    end
    curHeadErr
    
    if(delta(t+1) > 0.523)
        delta(t+1) = 0.523;
    end
    if(delta(t+1) < -0.523)
        delta(t+1) = -0.523;
    end 

    % Update next state
    x(:,t+1) = bicycle(x(:,t),v(t),delta(t+1),l,dt);
    %noise = [normrnd(0, 0.02) ; normrnd(0, 0.02) ; normrnd(0,0.01744)];
    %x(:,t+1) =  x(:,t+1)+ noise;
    
end


%% Create figure of motion
figure(1);
for t=1:2:n % For every second position of the robot
    clf;hold on; % clear the current figure
    
    drawcar(x(1,t),x(2,t),x(3,t),delta(t),0.1,1); % Draw the car
    
    plot(x(1,1:t), x(2,1:t), 'bx'); % Draw the path of the vehicle
    axis equal; % Use same scale on both axes
    %axis([-15 10 -25 5]); % Set the dimensions of the plot
    drawnow; % Draw the plot now instead of when Matlab feels like it
    if video 
        writeVideo(vidObj, getframe(gcf)); % Record the plot as a frame in the movie
    end
end

if video
    close(vidObj); % Finish making the movie file
end
