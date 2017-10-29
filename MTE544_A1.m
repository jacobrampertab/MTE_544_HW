2close all;
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
% angular velocities in rad/s
w1 = 1; w2 = -1; w3 = 0.1;
%% Initial State
x = 0; dx = 0; y = 0; dy = 0; theta = 0; dtheta = 0;
State = [x,y,dx,dy,theta,dtheta];
Omega = [w1,w2,w3];
X_graph= zeros(1,100);
Y_graph = zeros(1,100);
%% Outputting Motion
for step=1:N
    time = step*t;
    State = threewheel(State,Omega, r, l, time, sigma_theta, sigma_xy); 
    
    x = State(1); y = State(2); theta = State(5);
    draw_three_wheel_bot(x,y,theta,l,1);
    X_graph(step) = x; Y_graph(step) = y;

end
figure(2);
plot(X_graph, Y_graph,'-s');

