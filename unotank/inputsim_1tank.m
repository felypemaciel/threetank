function yout = inputsim_1tank(t,S,qin,mu20,Sp,g,x0)
    time_length = length(t);
    dt = t(2) - t(1);
    
    yout = zeros(time_length,3);
    yout(1,:) = 0.1;

    for i = 1:time_length - 1
        if i > 1
            x0 = y(end, :);
        end
        time_span = [dt*(i-1), dt*i];
        if i*dt >= 120
            qin = 20;
        end
        [~,y] = ode45(@(k,y)nonlinear1tank(k,y,S,qin,mu20,Sp,g), time_span, x0);
        yout(i+1,:) = y(end, :);
    end
end