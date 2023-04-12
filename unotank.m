%% Unotank
% Author: Felype Maciel (@felypemaciel)
% Modelling of a control system of a single water tank with constant
% outflow

close all;

% model parameters
r = 6.8/2;
S = pi*r*r;       % tank cross sectional area (cm2)
Sp = 0.2507;      % inter tank cross sectional area (cm2)

mu = 0.5;           % outflow coefficients
mu20 = 0.675;

qmax = 33.4;     % maximum flowrate (cm3/s)
lmax = 25;        % maximum level (cm)

g = 981;            % gravity (cm/s2)

% inputs
qin = 35;       % pump 1 flow (cm3/s)

% initial conditions
x0 = 2;           % first tank (cm)

trange = 0:0.1:5000;      % time range

% nonlinear model
[t, xnl] = ode45(@(t,x)nonlinear1tank(t,x,S,10,mu20,Sp,g), trange, x0);

plot(trange,xnl)
title('Open Loop')
xlabel('time (s)')
ylabel('height (cm)')

xss = fsolve(@(x)nonlinear1tank(t, x, qin, S, mu20, Sp, g),x0);

syms x1 u1 g mu20 S Sp 
q20 = mu20*Sp*sqrt(2*g*x1);
f = 1/S*(u1 - q20);
x = x1;
u = u1;

A_ss = jacobian(f,x);
B_ss = jacobian(f,u);

S = pi*r*r;       % tank cross sectional area (cm2)
Sp = 0.2507;      % inter tank cross sectional area (cm2)
mu20 = 0.675;
g = 981;            % gravity (cm/s2)
x1 = xss;
u1 = 35;

A = eval(A_ss);
B = eval(B_ss);
C = 1;
D= 0;

function dxdt = nonlinear1tank(t,x,S,qin,mu20,Sp,g)
    q20 = mu20*Sp*sqrt(2*g*x(1));
    dxdt = 1/S*(qin - q20);
end