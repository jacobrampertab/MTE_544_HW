function H= evaluate_H(X, u, T, r)
% X - previous state (or previous state prediction)
% T - time step
% r - wheel radius
% l - robot body radius
% u - current input (w1, w2, w3)
H = [1 0 0;
     0 1 0;
     0 0 1;
    ];