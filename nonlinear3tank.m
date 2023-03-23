function dxdt = nonlinear3tank(t,x,S,Sp,mu,mu20,g,q1,q2)
    
    % outflow between tanks
    q13 = mu*Sp*sqrt(2*g*(x(1)-x(3)));      % from tank1 to tank3
    q32 = mu*Sp*sqrt(2*g*(x(3)-x(2)));      % from tank3 to tank2
    q20 = mu20*Sp*sqrt(2*g*x(2));           % from tank2 to output
    
    % differential equations
    dxdt(1,1) = (1/S)*(q1 - q13);           % tank1           
    dxdt(2,1) = (1/S)*(q2 + q32 - q20);     % tank2
    dxdt(3,1) = (1/S)*(q13 - q32);          % tank3
end