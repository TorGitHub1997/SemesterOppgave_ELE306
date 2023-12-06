
close all;
clear;

% sl_drivepose corkes simulink
sl_drivepose;

x0 = [7 7 0];  % Initial pose of the robot [x, y, theta]
xg = [10 7 pi/2];  % Can symbolize the latitude and longitude in the ocean [x, y, theta]

r = sim("sl_drivepose", "StopTime", "10");
q = r.find('y');

Kp = 0.1;
error = xg(1:2) - [q(end, 1), q(end, 2)];  % Assuming q contains the final robot pose
control = Kp * error;
red = [1 0 0];
green = [0 1 0];
% plotting our results 
figure;
plot(q(:, 1), q(:, 2), 'LineWidth', 2);
hold on;
plot(x0(1), x0(2), 'ro', 'MarkerSize', 6, 'Color',red);
plot(xg(1), xg(2), 'ro', 'MarkerSize', 6,'Color',green);
quiver(q(end, 1), q(end, 2), control(1), control(2), 'r', 'LineWidth', 2);
title('Mobile Boat, To Pose');
xlabel('Latitude');
ylabel('Longitude');
legend('Robot Trajectory', 'Latitude and Longitude of robot');


bdclose('sl_drivepose');

