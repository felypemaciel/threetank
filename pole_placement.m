clear; close all; clc;

% state-space matrices
A = [-1.17e-2, 0, 1.17e-2;
    0, -2.27e-2, 1.17e-2;
    1.17e-2, 1.17e-2, -2.34e-2];

B = [64.935, 0;
    0, 64.395;
    0, 0];

C = eye(3);

D = 0;

% open-loop eigenvalues
eig(A);

