close all;
%% Parameters
% Duration 
total_t = 15;
% Times step (s)
t = 0.1;
% Steps 
N = round(total_t / t);
% Wheel Radius (m)
r = 0.25;
% Robot center to wheel hub(m)
l = 0.3;
% Guassian Disturbance:
sigma_theta = 0.1*pi()/180; sigma_xy = 0.01;
R = [sigma_xy 0 0 0 0 0; %covariance on x position
     0 0 0 0 0 0; %no uncertainty on x velocity
     0 0 sigma_xy 0 0 0; %covariance on y position
     0 0 0 0 0 0; %no uncertainty on y velocity
     0 0 0 0 sigma_theta 0; %covariance on angular position
     0 0 0 0 0 0; %no uncertainty on angular velocity
    ];
% Measurement Model Variance
magnetometer_sigma = 10*pi()/180; %10 degrees in radians
GPS_sigma = 0.5; % meters
Q = [GPS_sigma 0 0;
     0 GPS_sigma 0;
     0 0 magnetometer_sigma; 
     ];
%% Rotation inputs
% wheel speeds: angular velocities in rad/s
w1 = 0.5; w2 = -1; w3 = 2;
%% Initial State
%theta: angular position of robot
x = 0; dx = 0; y = 0; dy = 0; theta = 0; dtheta = 0;
State_true = [x,dx,y,dy,theta,dtheta];
State_calculated = [x,dx,y,dy,theta,dtheta]; % miu_t
Input = [w1,w2,w3];
% arrays to store robot path to graph at the end
X_graph= zeros(1,N);
Y_graph = zeros(1,N);
Sensor_x = zeros(1, N);
Sensor_y = zeros(1, N);
%% Determine G matrix
% TODO - make a seperate function to evaluate G matrix
% G = [-v1*sin(a1) -v2*sin(a2) -v3*sin(a3); 
%     v1*cos(a1) v2*cos(a2) v3*cos(a3)];
%% Outputting Motion
for step=1:N
    %1. evaluate true state
    State_true = evaluate_motion_model(State_true,Input, r, l, t, sigma_theta, sigma_xy);
    %2. evaluate measurement model based on state
    SensorModel = evaluate_sensor_model(State_true, magnetometer_sigma, GPS_sigma);
    %store sensor model values for graphing later on
    Sensor_x(step) = SensorModel(1); Sensor_y(step) = SensorModel(2);
    %% prediction update
    % Evaluate G_matrix
    G = evaluate_G(State_calculated, Input, t, r);
    State_prediction = evaluate_motion_model(State_calculated,Input, r, l, t, sigma_theta, sigma_xy);
    Covariance_prediction = zeros();
    % Evaluate miu (predicted state based on previous predicted state and
    % current input; based on motion model g(x,u))
    % Evalute Sigma predicted G*Sigma_prev*transpose(G) + R ; R is
    % convariance of motion model
    %% Measurement update
    %Evaluate H_matrix
    % H_pre
    %graphing
    x = State_true(1); y = State_true(2); theta = State_true(5);
    draw_three_wheel_bot(x,y,theta,l,1);
    X_graph(step) = x; Y_graph(step) = y;
    
end
figure(2);
plot(X_graph, Y_graph);
figure(3);
plot(Sensor_x, Sensor_y,'-s')

