clear; close all; clc;

A = [-1.17e-2, 0, 1.17e-2;
    0, -2.27e-2, 1.17e-2;
    1.17e-2, 1.17e-2, -2.34e-2];

B = [64.935, 0;
    0, 64.395;
    0, 0];

C = eye(3);

D = 0;

Q = [1  0   0;
     0  1   0;
     0  0   .1];

R = [1  0
     0  1];

K = lqr(A,B,Q,R);

sys = ss(A-B*K, B, C, D);

t = 0:0.1:1000;

% system setpoints
r = ones(length(t), 2);
r(:,1) = 30;                % tank 1
r(:,2) = 10;                % tank 2
% r(:,3) = 20;                % tank 3

y = lsim(sys, r(:,1:2), t);

u = r' - K*y';

plot(t,y(:,1)); 
hold on;
plot(t,y(:,2)); 
plot(t,y(:,3)); 

legend('tank1', 'tank2', 'tank3')

figure;
plot(t, u(1,:)');
hold on;
plot(t, u(2,:)');

S = 0.0154;     % tank cross sectional area (m2)
Sp = 5E-5;      % inter tank cross sectional area (m2)

mu = 0.5;       % outflow coefficients
mu20 = 0.675;

qmax = 1.2E-4;  % maximum flowrate (m3/s)
lmax = 0.6;     % maximum level (m)

g = 9.8;       % gravity (m/s2)

% inputs
q1 = 0.34E-4;   % pump 1 flow (m3/s)
q2 = 0.32E-4;   % pump 2 flow (m3/s)

% initial conditions
x1 = 0.1;       % first tank
x2 = 0.1;       % second tank
x3 = 0.1;       % third tank
x0 = [x1 x2 x3];

trange = 0:0.1:5000; 
sp1 = 0.45*ones(length(trange),1);      % setpoint for tank 1
sp2 = 0.25*ones(length(trange),1);      % setpoint for tank 2
sp3 = 0.35*ones(length(trange),1);      % setpoint for tank 3

limits = [0, qmax];

[yout, u, e] = lqr_control(trange,sp1,sp2,sp3, S,Sp,mu,mu20,g,q1,q2,x0,K,limits);

figure;
plot(trange,yout);
title('Simultaneous control')
xlabel('time (s)')
ylabel('Water level (m)')
legend('Tank 1', 'Tank 2', 'Tank 3')

figure;
plot(trange,u);
title('Control actions');
xlabel('time (s)')
ylabel('flowrate (m^3s^{-1})')
legend('pump1','pump2')