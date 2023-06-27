%% arduino_matlab
% This code is been used to obtain the data from Arduino, save it as a .mat
% or .csv file and to plot the data.

close all;

% delete(instrfind({'Port'},{'COM3'}));
delete(instrfind({'Port'},{'/dev/ttyACM2'}));

% s = serial('COM3', 'BaudRate', 9600);  % Replace 'COMx' with the appropriate port
s = serial('/dev/ttyACM2', 'BaudRate', 9600);  % Replace 'COMx' with the appropriate port
fopen(s);

dataVector = [];  % Initialize an empty vector
stopCondition = false;  % Termination condition flag
time = 0;

while time < 200
    if s.BytesAvailable > 0
        data = fscanf(s, '%f');  % Read the data from Arduino
        if length(data) == 4
            dataVector = [dataVector; data.'];  % Append the received data to the vector
            disp(data);  % Display the received data in MATLAB
        end
        time = dataVector(end,1);
    end
end

fclose(s);

% Save the data vector to a file
% save('setpoint_tracking.mat', 'dataVector');
% writematrix(dataVector,'setpoint_tracking.csv')

subplot(2,1,1);
plot(dataVector(:,1), dataVector(:,2), 'LineWidth', 1);
title('Step response');
xlabel('time (s)');
ylabel('water level (cm)');
grid;

subplot(2,1,2);
plot(dataVector(:,1), dataVector(:,3));
title('System input - Control action');
xlabel('time (s)');
ylabel('water flow (cm^3/s)');
grid;