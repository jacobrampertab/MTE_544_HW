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
%% Rotation inputs
% wheel speeds: angular velocities in rad/s
w1 = 1; w2 = -1; w3 = -1;
%% Initial State
%theta: angular position of robot
x = 0; dx = 0; y = 0; dy = 0; theta = 0; dtheta = 0;
State = [x,dx,y,dy,theta,dtheta];
Omega = [w1,w2,w3];
% arrays to store robot path to graph at the end
X_graph= zeros(1,100);
Y_graph = zeros(1,100);

%% Determine G matrix
% TODO - make a seperate function to evaluate G matrix
% G = [-v1*sin(a1) -v2*sin(a2) -v3*sin(a3); 
%     v1*cos(a1) v2*cos(a2) v3*cos(a3)];
%% Outputting Motion
for step=1:N
    time = step*t;
    %1. evaluate true state
    State = evaluate_motion_model(State,Omega, r, l, t, sigma_theta, sigma_xy);
    %2. evaluate measurement model based on state
    % evaluate_sensor_model(...)
    %% prediction update
    % Evaluate G_matrix
    % G_pred = evaluate_G(...) - note G is partial derivative wrt x of motion model g(x,u)
    % Evaluate miu (predicted state based on previous predicted state and
    % current input; based on motion model g(x,u))
    % Evalute Sigma predicted G*Sigma_prev*transpose(G) + R ; R is
    % convariance of motion model
    %% Measurement update
    %Evaluate H_matrix
    % H_pre
    %graphing
    x = State(1); y = State(2); theta = State(5);
    draw_three_wheel_bot(x,y,theta,l,1);
    X_graph(step) = x; Y_graph(step) = y;
    
end
figure(2);
plot(X_graph, Y_graph,'-s');

