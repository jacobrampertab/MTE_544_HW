function g = evaluate_little_g(X, u, T, r, l)
% X - previous state (or previous state prediction)
% T - time step
% r - wheel radius
% l - robot body radius
% u - current input (w1, w2, w3)
w1 = u(1); w2 = u(2); w3 = u(3);
k1 = (w3-w2)*r*0.866;
k2 = r*(w1-0.5*(w2+w3)*0.5);
g = [X(1) + T*(k1*cos(X(3)) - k2*sin(X(3)));
     X(2) + T*(k1*sin(X(3)) + k2*cos(X(3)));
     X(3) + T*(r/l*(w1+w2+w3));
    ];