function [A, B, C, D] = lin3tank(xss)
    syms S Sp mu mu20 g u1 u2 x1 x2 x3

    f1 = u1/S - mu*Sp*sqrt(2*g*(x1-x3))/S;
    f2 = u2/S + mu*Sp*sqrt(2*g*(x3-x2))/S - mu20*Sp*sqrt(2*g*x2)/S;
    f3 = mu*Sp*sqrt(2*g*(x1-x3))/S - mu*Sp*sqrt(2*g*(x3-x2))/S;

    f = [f1, f2, f3];
    x = [x1, x2, x3];
    u = [u1, u2];
    
    A_ss = jacobian(f,x);
    B_ss = jacobian(f,u);

    % model parameters
    S = 0.0154;     % tank cross sectional area (m2)
    Sp = 5E-5;      % inter tank cross sectional area (m2)

    mu = 0.5;       % outflow coefficients
    mu20 = 0.675;

    g = 9.8;       % gravity (m/s2)

    % inputs
    u1 = 0.34E-4;   % pump 1 flow (m3/s)
    u2 = 0.32E-4;   % pump 2 flow (m3/s)

    % state variables
    x1 = xss(1);
    x2 = xss(2);
    x3 = xss(3);

    A = eval(A_ss);
    B = eval(B_ss);
    C = eye(3);
    D = 0;