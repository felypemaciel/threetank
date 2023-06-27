%% Unotank
% Author: Felype Maciel (@felypemaciel)
% Modelling of a control system of a single water tank with constant
% outflow

clc; clear; close all;

% model parameters
r = 6.63/2;         % tank radius (cm)
rp = 0.565/2;       % pipe radius (cm)
S = pi*r*r;       % tank cross sectional area (cm2)
Sp = pi*rp*rp;      % inter tank cross sectional area (cm2)

% mu = 0.5;           % outflow coefficients
mu = 0.515;

qmax = 27.8;     % maximum flowrate (cm3/s)
lmax = 25;        % maximum level (cm)

g = 981;            % gravity (cm/s2)

% inputs
qin = 15;       % pump 1 flow (cm3/s)

% initial conditions
x0 = 1;           % first tank (cm)

trange = 0:0.1:1500;      % time range

% nonlinear model
[t, xnl] = ode45(@(t,x)nonlinear1tank(t,x,S,qin,mu,Sp,g), trange, x0);

figure;
plot(trange,xnl)
title('Open Loop')
xlabel('time (s)')
ylabel('height (cm)')
grid;
axis([0 200 0 inf])

xss = fsolve(@(x)nonlinear1tank(t, x, S, qin, mu, Sp, g),x0);

syms x1 u1 g mu S Sp 
q20 = mu*Sp*sqrt(2*g*x1);
f = 1/S*(u1 - q20);
x = x1;
u = u1;

A_ss = jacobian(f,x);
B_ss = jacobian(f,u);

S = pi*r*r;       % tank cross sectional area (cm2)
Sp = pi*rp*rp;      % inter tank cross sectional area (cm2)
mu = 0.515;
g = 981;            % gravity (cm/s2)
x1 = xss;
u1 = qin;

A = eval(A_ss);
B = eval(B_ss);
C = 1;
D = 0;

y_insim = inputsim_1tank(t,S,qin,mu,Sp,g,x0);
figure; 
plot(trange, y_insim);
title('Step Response');
ylabel('height (cm)');
xlabel('time (s)');
grid;
axis([0 350 0 15])

sp = 10*ones(length(trange),1);
sp(1200:end) = 10;
sp(4000:end) = 15;
sp(6500:end) = 10;
sp(8000:end) = 5;
sp(10500:end) = 10;
[yout, u, e, I, ie] = control_1tank(t,sp,S,Sp,mu,g,qin,x0,[9.0625, 30.74, 0],[0 qmax]);
figure; 
plot(trange, yout, 'LineWidth', 1);
hold on;
plot(trange, sp, '--', 'linewidth', 1);
title('Step Response');
ylabel('height (cm)');
xlabel('time (s)');
grid;

figure; plot(trange, u)