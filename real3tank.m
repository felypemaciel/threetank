clc; close all; clear;

% model parameters
S = 4.513E-3;       % tank cross sectional area (m2)
Sp = 2.507E-5;      % inter tank cross sectional area (m2)

mu = 0.5;           % outflow coefficients
mu20 = 0.675;

qmax = 5E-5;        % maximum flowrate (m3/s)
lmax = 0.26;        % maximum level (m)

g = 9.8;            % gravity (m/s2)


% inputs
q1 = 0.15E-4;       % pump 1 flow (m3/s)
q2 = 0.1E-4;        % pump 2 flow (m3/s)

% initial conditions
x1 = 0.1;           % first tank
x2 = 0.1;           % second tank
x3 = 0.1;           % thrid tank
x0 = [x1 x2 x3];

trange = 0:0.1:5000;      % time range

% nonlinear model
[t, xnl] = ode45(@(t,x)nonlinear3tank(t,x,S,Sp,mu,mu20,g,q1,q2), trange, x0);

plot(t, xnl,'linewidth',1)
title('Open-loop system - constant inputs');
xlabel('time (s)')
ylabel('Water level (m)')
legend('Tank 1', 'Tank 2', 'Tank 3')
grid;

% equilibrium points
xss = fsolve(@(x)nonlinear3tank(t,x,S,Sp,mu,mu20,g,q1,q2),x0);

% state space model
[A, B, C, D] = lin3tank(xss);

% controllability test
% syms lambda
% rank([A - lambda*eye(3), B])

% observability test
% rank([A'-lambda*eye(3), C'])

%% Control simulations
% % Controller for tank 1
% sp = 0.4*ones(length(trange),1);      % setpoint
% sp(10000:20000) = 0.5;
% initial = x0;                       % initial conditions
% controller1 = [0.28, 222, 0];        % controller parameters
% limits = [0, qmax];                 % system's input limits
% 
% [yout, u1, e] = control_sim_t1(trange,sp,S,Sp,mu,mu20,g,q1,q2,initial,controller1,limits);
% 
% figure;
% plot(trange,yout,trange,sp,'--');
% title('Controller for Tank 1 - Active');
% xlabel('time (s)')
% ylabel('Water level (m)')
% legend('tank1','tank2','tank3','setpoint');
% 
% u2 = q2*ones(1,length(trange));
% 
% 
% figure; 
% plot(trange, u1, trange, u2);
% title("Control actions for tank 1 controller");
% xlabel('time (s)')
% ylabel('flowrate (m^3s^{-1})')
% legend('pump1','pump2')

% % Controller for tank 3
% sp = 0.4*ones(length(trange),1);      % setpoint
% sp(10000:20000) = 0.5;
% 
% Kc = @(tauc) 246/(0.289*tauc);
% kc = Kc(32768)
% controller2 = [kc, 246, 0];        % controller parameters
% [yout, u2, e] = control_sim_t3(trange,sp,S,Sp,mu,mu20,g,q1,q2,initial,controller2,limits);
% 
% figure;
% plot(trange,yout,trange,sp,'--');
% title('Controller for Tank 3 only');
% xlabel('time (s)')
% ylabel('Water level (m)')
% legend('tank1','tank2','tank3','setpoint');
% 
% u1 = q1*ones(1,length(trange));
% 
% figure; 
% plot(trange, u2, trange, q1);
% title("Control actions for tank 3 controller");
% xlabel('time (s)')
% ylabel('flowrate (m^3s^{-1})')
% legend('pump2','pump1')

trange = 0:0.1:5000;                    % time range

sp1 = 0.25*ones(length(trange),1);      % setpoint for tank 1
sp1(20000:end) = 0.15;
sp2 = 0.15*ones(length(trange),1);      % setpoint for tank 2

% Kc = @(tauc) 246/(0.289*tauc);
% kc = Kc(65536);

ctrl1 = [0.052, 54.8, 0];                 % controller 1 coefficients
ctrl2 = [7.54e-3, 66.9, 0];                % controller 2 coefficients

limits = [0, qmax];                     % pumps flowrates limits

% simulation
[yout, u, e] = control_sim_t13(trange,sp1,sp2,S,Sp,mu,mu20,g,q1,q2,x0,ctrl1,ctrl2,limits);

figure;
plot(trange,yout,'linewidth',1);
title('Simultaneous control')
xlabel('time (s)')
ylabel('Water level (m)')
legend('Tank 1', 'Tank 2', 'Tank 3')
grid;

figure;
plot(trange,u,'linewidth',1);
title('Control actions');
xlabel('time (s)')
ylabel('flowrate (m^3s^{-1})')
legend('pump1','pump2')
grid;