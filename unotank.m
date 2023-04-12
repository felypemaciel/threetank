%% Unotank
% Author: Felype Maciel (@felypemaciel)
% Modelling of a control system of a single water tank with constant
% outflow

close all;

% model parameters
S = 4.513E-3;       % tank cross sectional area (m2)
Sp = 2.507E-5;      % inter tank cross sectional area (m2)

mu = 0.5;           % outflow coefficients
mu20 = 0.675;

qmax = 3.34E-5;     % maximum flowrate (m3/s)
lmax = 0.26;        % maximum level (m)

g = 9.8;            % gravity (m/s2)

% inputs
qin = 3.5E-5;       % pump 1 flow (m3/s)
qout = 0.8E-4;

% initial conditions
x0 = 0.05;           % first tank

trange = 0:0.1:5000;      % time range

% nonlinear model
[t, xnl] = ode45(@(t,x)nonlinear1tank(t,x,S,qin,mu20,Sp,g), trange, x0);

plot(trange,xnl)

function dxdt = nonlinear1tank(t,x,S,qin,mu20,Sp,g)
    q20 = mu20*Sp*sqrt(2*g*x(1));
    dxdt = 1/S*(qin - q20);

end