function yout = in_sim(t,S,Sp,mu,mu20,g,q1,q2,x0)
    time_length = length(t);
    dt = t(2) - t(1);
    
    yout = zeros(time_length,3);
    yout(1,:) = 0.1;

    for i = 1:time_length - 1
        if i > 1
            x0 = y(end, :);
        end
        time_span = [dt*(i-1), dt*i];
        if i*dt >= 3500
            q2 = 0.42E-4;
        end
        [~,y] = ode45(@(k,y)nonlinear3tank(k,y,S,Sp,mu,mu20,g,q1,q2), time_span, x0);
        yout(i+1,:) = y(end, :);
    end
end