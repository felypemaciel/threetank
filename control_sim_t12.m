function [yout, u, e] = control_sim_t13(t,sp1,sp2,S,Sp,mu,mu20,g,q1,q2,initial,ctrl1,ctrl2,limits)
    % controller 1 coefficients
    kC1 = ctrl1(1);             % proportional
    tauI1 = ctrl1(2);           % integral
    tauD1 = ctrl1(3);           % derivative

    % controller 2 coefficients
    kC2 = ctrl2(1);             % proportional
    tauI2 = ctrl2(2);           % integral
    tauD2 = ctrl2(3);           % derivative

    tl = length(t);             % time length
    dt = t(2) - t(1);           % dt

    u = zeros(2, tl);           % controller output / system's input
    yout = zeros(tl, 3);        % system's output
    e = zeros(2, tl);           % error
    ie = zeros(2, tl);          % integral of the error
    dyout = zeros(2,tl);        % derivative of the output

    % for controller 1
    P1 = zeros(1,tl);           % proportional vector
    I1 = zeros(1,tl);           % integral vector
    D1 = zeros(1,tl);           % derivative vector

    % for controller 2
    P2 = zeros(1,tl);           % proportional vector
    I2 = zeros(1,tl);           % integral vector
    D2 = zeros(1,tl);           % derivative vector

    u_ll = limits(1);           % controller output lower limit
    u_ul = limits(2);           % controller output upper limit

    yout(1,:) = initial(1);     % outputs initial condition

    T21 = -0.3967;              % decoupler y2,u1
    T12 = -0.3;                 % decoupler y1,u2

    for i = 1:tl-1

        e(1,i) = sp1(i,1) - yout(i,1); % error 1
        e(2,i) = sp2(i,1) - yout(i,2); % error 2

        if i > 1
            % integral of the error
            ie(1,i) = ie(1,i+1) + e(1,i)*dt;
            ie(2,i) = ie(2,i+1) + e(2,i)*dt;
            
            % derivative of the error
            dyout(1,i) = (yout(i,1)-yout(i-1,1))/dt;
            dyout(2,i) = (yout(i,2)-yout(i-1,2))/dt;

            initial = y(end,:);
        end
            
        % controller 1 output calculation
        P1(i) = kC1*e(1,i);                 % proportional value at isntant i
        I1(i) = kC1/tauI1 * ie(1,i);        % integral value at instant i
        D1(i) = -kC1*tauD1*dyout(1,i);      % derivative value at isntant i
        u(1,i) = P1(i) + I1(i) + D1(i);     % controller output at instant i
            
        % controller 2 output calculation 
        P2(i) = kC2*e(2,i);                 % proportional value at isntant i
        I2(i) = kC2/tauI2 * ie(2,i);        % integral value at instant i
        D2(i) = -kC2*tauD2*dyout(2,i);      % derivative value at isntant i
        u(2,i) = P2(i) + I2(i) + D2(i);     % controller output at instant i

        % system's input lower limit control
        [u(1,i),u(2,i),e(1,i),e(2,i),ie(1,i),ie(2,i)] = lim_test(u(1,i),u(2,i),e(1,i),e(2,i),ie(1,i),ie(2,i));
         
        ts = [dt*(i-1), dt*i];      % time span

        % decouplers actions
        U21 = u(1,i)*T21;                   % decoupler y2,u1 output
        U12 = u(2,i)*T12;                   % decoupler y1,u2 output

        % control actions
        u(1,i) = u(1,i) + U12;
        u(2,i) = u(2,i) + U21;

        % system's input limits
        [u(1,i),u(2,i),e(1,i),e(2,i),ie(1,i),ie(2,i)] = lim_test(u(1,i),u(2,i),e(1,i),e(2,i),ie(1,i),ie(2,i));
        
        q1 = u(1,i);                % inflow on tank 1
        q2 = u(2,i);                % inflow on tank 2
        
        [l,y] = ode45(@(k,y)nonlinear3tank(k,y,S,Sp,mu,mu20,g,q1,q2),ts,initial);
        yout(i+1,:) = y(end,:);
    end

    function [u1,u2,e1,e2,ie1,ie2] = lim_test(u1,u2,e1,e2,ie1,ie2)
        % lower limit
        if u1 < u_ll
            u1 = u_ll;
            ie1 = ie1 - e1*dt;
        end
        if u2 < u_ll
            u2 = u_ll;
            ie2 = ie2 - e2*dt;
        end

        % upper limit
        if u1 > u_ul
            u1 = u_ul;
            ie1 = ie1 - e1*dt;
        end
        if u2 > u_ul
            u2 = u_ul;
            ie2 = ie2 - e2*dt;
        end
    end

end