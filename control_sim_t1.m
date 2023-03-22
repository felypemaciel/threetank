function [yout, u, e] = control_sim_t1(t,sp,S,Sp,mu,mu20,g,q1,q2,initial,controller,limits)
    % controller coefficients
    kC = controller(1);     % proportional
    tauI = controller(2);   % integral
    tauD = controller(3);   % derivative

    tl = length(t);     % time length
    dt = t(2) - t(1);   % dt

    u = zeros(1, tl);           % controller output / system's input
    yout = zeros(tl, 3);        % system's output
    e = zeros(1, tl);           % error
    ie = zeros(1, tl);          % integral of the error
    dyout = zeros(1,tl);        % derivative of the output
    P = zeros(1,tl);            % proportional vector
    I = zeros(1,tl);            % integral vector
    D = zeros(1,tl);            % derivative vector

    u_ll = limits(1);           % controller output lower limit
    u_ul = limits(2);           % controller output upper limit

    yout(1,:) = initial(1);       % output initial condition

    for i = 1:tl-1
        e(i) = sp(i,1) - yout(i,1); % error
        if i > 1
            ie(i) = ie(i+1) + e(i)*dt;
            dyout(i) = (yout(i)-yout(i-1))/dt;
            initial = y(end,:);
        end
            P(i) = kC*e(i);                 % proportional value at isntant i
            I(i) = kC/tauI * ie(i);         % integral value at instant i
            D(i) = -kC*tauD*dyout(i);       % derivative value at isntant i
            u(i) = P(i) + I(i) + D(i);      % controller output at instant i

            % system's input lower limit control
            if u(i) < u_ll
                u(i) = u_ll;
                ie(i) = ie(i) - e(i)*dt;
            end

            % system's input upper limit control
            if u(i) > u_ul
                u(i) = u_ul;
                ie(i) = ie(i) - e(i)*dt;
            end
    
        ts = [dt*(i-1), dt*i];      % time span
        % q1 = u(i);      % inflow on tank 1
        q2 = u(i);      % inflow on tank 2

        [l,y] = ode45(@(k,y)nonlinear3tank(k,y,S,Sp,mu,mu20,g,q1,q2),ts,initial);
        yout(i+1,:) = y(end,:);
    end
end