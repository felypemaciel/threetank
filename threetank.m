% model parameters
S = 0.0154;     % tank cross sectional area (m2)
Sp = 5E-5;      % inter tank cross sectional area (m2)

mu = 0.5;       % outflow coefficients
mu20 = 0.675;

qmax = 1.2E-4;  % maximum flowrate (m3/s)
lmax = 0.6;     % maximum level (m)

g = 9.81;       % gravity (m/s2)

% inputs
q1 = 0.34E-4;   % pump 1 flow (m3/s)
q2 = 0.32E-4;   % pump 2 flow (m3/s)

% initial conditions
x1 = 0.1;       % first tank
x2 = 0.1;       % second tank
x3 = 0.1;       % thrid tank
x0 = [x1 x2 x3];

trange = 0:3000;      % time range

% nonlinear model
[t, xnl] = ode45(@(t,x)nonlinear_3tank(t,x,S,Sp,mu,mu20,g,q1,q2), trange, x0);

plot(t, xnl)
title('Open-loop system - constant inputs');
xlabel('time (s)')
ylabel('Water level (m)')
legend('Tank 1', 'Tank 2', 'Tank 3')

