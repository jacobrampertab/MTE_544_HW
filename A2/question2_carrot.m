%% Trajectory tracking

video = false;

%% Create AVI movie object
if video
    vidObj = VideoWriter('car.avi'); % Define filename
    vidObj.Quality = 100; % Set quality
    vidObj.FrameRate = 20; % Set framerate
    open(vidObj); % Open video object
end


%% Simulation Variables
% Fixed vehicle parameters
velocity = 3; % Speed
delta_max = 30*pi/180; % max steering angle
k = 10; % Gain
ks = -1; % Damping
robot_length = 0.34; % Car length
% TODO length explodes the driving trajectory below 0.34, must resolve
r = 2; % Carrot Distance

% Desired trajectory as a plot of points. The plot of points defines
% multiple line segments all joined together. The vehicle will first follow
% along the first line segment. After it is has driven past the length of
% said line segment, it defines a new line segment to follow by taking the
% next point.
traj_points = [20,0;
               20,5;
               0, 5;
               0, 0];

traj_point_counter = 1; % Keep track of where we are in the trajectory
           
% Initial conditions in [x y heading]
x0 = [0 0 0]; 

% Simulation time
Tmax = 20;  % End point
dt =0.01; % Time step
T = 0:dt:Tmax; % Time vector

% Simulation setup
xd = zeros(length(T)-1,3); % Derivative of state ([edot psidot])
x = zeros(length(T),3);  % State ([e psi] 
x(1,:) = x0; % Initial condition
delta = zeros(length(T),1); % Steering angles


%% Main Loop
for i=1:length(T)-1
    % The desired trajectory is a line segment consisting of 2 points from
    % the desired trajectory
    end_point = traj_points(traj_point_counter+1, :);
    start_point = traj_points(traj_point_counter, :);
    traj_angle = atan2(end_point(2) - start_point(2), end_point(1) - start_point(1));
    
    carrot_pos = calculate_carrot(start_point, end_point, x(i,1:2), r);
    [crosstrack_error, next_point] = distanceToLineSegment(start_point,carrot_pos,x(i,1:2));
    
    % Calculate steering angle
    delta(i) = max(-delta_max, min(delta_max, angleWrap(traj_angle - x(i,3))+ atan2(-k*crosstrack_error,ks+velocity)));
    % State derivatives
    xd(i,1) = velocity*cos(x(i,3));
    xd(i,2) = velocity*sin(x(i,3));
    xd(i,3) = velocity*tan(delta(i)/robot_length);
    
    % Noise
    %noise = [normrnd(0, 0.02), normrnd(0, 0.02), normrnd(0,0.01744)];
    noise = [0,0,0];
    
    % State update
    x(i+1,1) = x(i,1)+dt*xd(i,1)+noise(1);
    x(i+1,2) = x(i,2)+dt*xd(i,2)+noise(2);
    x(i+1,3) = x(i,3)+dt*xd(i,3)+noise(3);
    
    % angle wrap the heading
    x(i+1,3) = angleWrap(x(i+1,3));
    
 
    % Check if we have travelled the distance of the line segment. 
    % If we have, then get the next point
    if (next_point == 1)
        traj_point_counter = traj_point_counter+1;
        disp("reached point");
        if (traj_point_counter == length(traj_points(:,1)))
            break;
        end
    end
end


%% Plotting
% Plotting the trajectory of the vehicle
figure(1);clf; hold on;
plot(x(1:i,1),x(1:i,2),'b-');
scatter(traj_points(:,1), traj_points(:,2),10,'r');

for t=1:30:i
      drawbox(x(t,1),x(t,2),-x(t,3),.5,1);
end
xlabel('x (m)')
ylabel('y (m)')
axis equal
hold off;

% Alternative trajectory plotting
figure(2); hold on;
for t=1:60:i % For every second position of the robot
    drawcar(x(t,1),x(t,2),x(t,3),delta(t),0.1,2); % Draw the car
    
    plot(x(1:t,1), x(1:t,2), 'bx'); % Draw the path of the vehicle
    axis equal; % Use same scale on both axes
    drawnow; % Draw the plot now instead of when Matlab feels like it
    if video 
        writeVideo(vidObj, getframe(gcf)); % Record the plot as a frame in the movie
    end
end




