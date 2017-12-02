function G = evaluate_G(X, u, T, r)
% X - previous state (or previous state prediction)
% T - time step
% r - wheel radius
% l - robot body radius
% u - current input (w1, w2, w3)
% to be implemented
w1 = u(1); w2 = u(2); w3 = u(3);
k1 = (w3-w2)*r*0.866;
k2 = r*(w1-0.5*(w2+w3)*0.5);
G = [1, 0, T*(-sin(X(3))*k1 - cos(X(3))*k2);
     0, 1, T*(cos(X(3))*k1 - sin(X(3))*k2);
     0, 0, 1;
    ];