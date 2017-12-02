function G = evaluate_G(X, u, T, r)
% X - previous state (or previous state prediction)
% T - time step
% r - wheel radius
% l - robot body radius
% u - current input (w1, w2, w3)
% to be implemented
theta = X(5);
w1 = u(1); w2 = u(2); w3 = u(3);
vx = (w3-w2)*r*0.866;
vy = r*(w1-0.5*(w2+w3)*0.5);
     
     x_new = x_prev + T*x_dot
     %x %x_dot %y %y_dot %theta %theta_dot
G = [1, T, 0, 0,                              0, 0; %x
     0, 0, 0, 0, -sin(theta)*vx - cos(theta)*vy, 0; %x_dot
     0, 0, 1, T,                              0, 0; %y
     0, 0, 0, 0,  cos(theta)*vx - sin(theta)*vy, 0; %y_dot
     0, 0, 0, 0,                              1, T; %theta
     0, 0, 0, 0,                              0, 0; %theta_dot
     ];