function g = evaluate_little_g(X, u, T, r, l)
% X - previous state (or previous state prediction)
% T - time step
% r - wheel radius
% l - robot body radius
% u - current input (w1, w2, w3)
w1 = u(1); w2 = u(2); w3 = u(3);
k1 = (w3-w2)*r*0.866;
k2 = r*(w1-0.5*(w2+w3)*0.5);
g = [
    X(1) + T*X(2);
    k1*cos(X(5)) - k2*sin(X(5));
    X(3) + T*X(4);
    k1*sin(X(5)) + k2*cos(X(5));
    X(5) + T*X(6);
    r/l*(w1+w2+w3);
    ];