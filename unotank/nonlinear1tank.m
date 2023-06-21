function dxdt = nonlinear1tank(t,x,S,qin,mu20,Sp,g)
    q20 = mu20*Sp*sqrt(2*g*x(1));
    dxdt = 1/S*(qin - q20);
end