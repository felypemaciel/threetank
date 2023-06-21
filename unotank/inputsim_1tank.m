%% inputsim_1tank
% Here there is just a simple input step test. 
% The system will receive the input qin untill t = stime seconds, when the
% input becomes = step_u.

function yout = inputsim_1tank(t,S,qin,mu20,Sp,g,x0)
    time_length = length(t);
    dt = t(2) - t(1);

    stime = 120;        % step time (s)
    step_u = 20;        % new input value (cm3/s)
    
    yout = zeros(time_length,3);
    yout(1,:) = 0.1;        % initial condition

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