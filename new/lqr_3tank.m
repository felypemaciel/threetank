clear; close all; clc;

A = [-1.17e-2, 0, 1.17e-2;
    0, -2.27e-2, 1.17e-2;
    1.17e-2, 1.17e-2, -2.34e-2];

B = [64.935, 0;
    0, 64.395;
    0, 0];

C = eye(3);

D = 0;

x0 = [3;
      1]

Q = [1  0   0;
     0  1   0;
     0  0   1];

R = [1  0
     0  1];

K = lqr(A,B,Q,R);

sys = ss(A-B*K, B, C, D);

t = 0:0.1:1000;

% system setpoints
r = ones(length(t), 2);
r(:,1) = 25;                % tank 1
r(:,2) = 10;                % tank 2

y = lsim(sys, r, t);

r(:,3) = 0;

u = K*r' -K*y';


subplot(2,1,1);
plot(t,y(:,1)); 
hold on;
plot(t,y(:,2)); 
plot(t,y(:,3)); 
legend('tank1', 'tank2', 'tank3')
xlabel('Time (s)');
ylabel('Water level (cm)')
title('System response')

subplot(2,1,2)
plot(t, u(1,:)');
hold on;
plot(t, u(2,:)');
xlabel('Time (s)');
ylabel('Water flow rate (cm^3/s)')
title('System inputs')

sim("sim_lqr3tank.slx");

subplot(4,1,1);
plot(out.x.Time, out.x.Data(:,1));