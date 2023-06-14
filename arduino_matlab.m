close all;

% delete(instrfind({'Port'},{'COM3'}));
delete(instrfind({'Port'},{'/dev/ttyACM2'}));

% s = serial('COM3', 'BaudRate', 9600);  % Replace 'COMx' with the appropriate port
s = serial('/dev/ttyACM2', 'BaudRate', 9600);  % Replace 'COMx' with the appropriate port
fopen(s);

dataVector = [];  % Initialize an empty vector
stopCondition = false;  % Termination condition flag
time = 0;

while time < 320
    if s.BytesAvailable > 0
        data = fscanf(s, '%f');  % Read the data from Arduino
        if length(data) == 3
            dataVector = [dataVector; data.'];  % Append the received data to the vector
            disp(data);  % Display the received data in MATLAB
        end
        time = dataVector(end,1);
    end
end

fclose(s);

% Save the data vector to a file
% save('control_action.mat', 'dataVector');
% writematrix(dataVector,'control_action.csv')

subplot(2,1,1);
plot(dataVector(:,1), dataVector(:,2));
title('Ultrasonic sensor distance');
xlabel('time (s)');
ylabel('distance (cm)');
grid;

subplot(2,1,2);
plot(dataVector(:,1), dataVector(:,3));
title('System input - Control action');
xlabel('time (s)');
ylabel('distance (cm)');
grid;