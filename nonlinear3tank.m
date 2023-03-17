function dxdt = nonlinear_3tank(t,x,S,Sp,mu,mu20,g,q1,q2)
    dxdt(1,1) = q1/S - mu*Sp*sqrt(2*g*(x(1)-x(3)))/S;
    dxdt(2,1) = q2/S + mu*Sp*sqrt(2*g*(x(3)-x(2)))/S - mu20*Sp*sqrt(2*g*x(2))/S;
    dxdt(3,1) = mu*Sp*sqrt(2*g*(x(1)-x(3)))/S - mu*Sp*sqrt(2*g*(x(3)-x(2)))/S;
end