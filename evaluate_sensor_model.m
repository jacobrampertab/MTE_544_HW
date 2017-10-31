function SENSOR = evaluate_sensor_model(State,sigma_theta, sigma_xy)
%State - [x,y,theta]
%Sigma theta - magnetometer noise
%Sigma xy - gps noise
angular_disturbance = normrnd(0,sigma_theta);
x_disturbance = normrnd(0,sigma_xy);
y_disturbance = normrnd(0,sigma_xy);
x_new = State(1) + x_disturbance;
y_new = State(2) + y_disturbance;
angle_new = State(3) + angular_disturbance + 9.7;
SENSOR = [x_new, y_new, angle_new];