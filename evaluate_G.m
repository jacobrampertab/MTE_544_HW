function G_pred = evaluate_G(G, X, u, T, r)
% X - previous state (or previous state prediction)
% T - time step
% r - wheel radius
% l - robot body radius
% u - current input (w1, w2, w3)
% to be implemented
theta = X(5);
w1 = u(1); w2 = u(2); w3 = u(3);
k1 = (w3-w2)*r*0.866;
k2 = r*(w1-0.5*(w2+w3)*0.5;
G = [1, T, 0, 0,                              0, 0;
     0, 0, 0, 0, -sin(theta)*k1 - cos(theta)*k2, 0;
     0, 0, 1, T,                              0, 0;
     0, 0, 0, 0,  cos(theta)*k1 - sin(theta)*k2, 0;
     0, 0, 0, 0,                              1, T;
     0, 0, 0, 0,                              0, 0;
     ]
G_pred = G;