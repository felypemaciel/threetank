function [yout, u, e] = lqr_control(t,sp1,sp2,sp3,S,Sp,mu,mu20,g,q1,q2,initial,K,limits)
    
    tl = length(t);             % time length
    dt = t(2) - t(1);           % dt

    u = zeros(3, tl);           % controller output / system's input
    yout = zeros(tl, 3);        % system's output
    e = zeros(3, tl);           % error

    u_ll = limits(1);           % controller output lower limit
    u_ul = limits(2);           % controller output upper limit

    yout(1,:) = initial(1);     % outputs initial condition

    for i = 1:tl-1

        e(1,i) = sp1(i,1) - yout(i,1);  % error 1
        e(2,i) = sp2(i,1) - yout(i,2);  % error 2
        e(3,i) = sp3(i,1) - yout(i,3);  % error 3

        in = [sp1(i,1);
            sp2(i,1)] -K*[ yout(i,1);
                 yout(i,2);
                 yout(i,1)];

        if i > 1
            initial = y(end,:);
        end
            
        % controller 1 output calculation
        u(1,i) = in(1);     % controller output at instant i
            
        % controller 2 output calculation 
        u(2,i) = in(2);     % controller output at instant i

        % system's input limits
        [u(1,i),u(2,i)] = lim_test(u(1,i),u(2,i));
        
        q1 = u(1,i);                % inflow on tank 1
        q2 = u(2,i);                % inflow on tank 2

        ts = [dt*(i-1), dt*i];      % time span

        
        [l,y] = ode45(@(k,y)nonlinear3tank(k,y,S,Sp,mu,mu20,g,q1,q2),ts,initial);
        yout(i+1,:) = y(end,:);
    end

    function [u1,u2] = lim_test(u1,u2)
        % lower limit
        if u1 < u_ll
            u1 = u_ll;
        end
        if u2 < u_ll
            u2 = u_ll;
        end

        % upper limit
        if u1 > u_ul
            u1 = u_ul;
        end
        if u2 > u_ul
            u2 = u_ul;
        end
    end

end