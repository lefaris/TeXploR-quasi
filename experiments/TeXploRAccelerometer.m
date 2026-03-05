%Read in Vicon accelerometer data for TeXploR

opts = detectImportOptions('Test 1.csv');
M = readmatrix('Test 1.csv','Range','A2:H19131');

%Read arc 1 data:
time = M(1:19125, 1);
accel_x = M(1:19125, 6);
accel_y = M(1:19125, 7);
accel_z = M(1:19125, 8);
x = zeros(19125);
for i = 1:19125
    x(i) = i;
end



% Initial plotting
figure
for j = 1:10:4500
    plot(x(j), accel_x(j), 'b+', x(j), accel_y(j), 'r+', x(j), accel_z(j), 'g+')
    hold on
end
ylim([-15 15])
legend('Acceleration in X', 'Acceleration in Y', 'Acceleration in Z')
title('Accelerometer Data for Shifting Mass 1 During Smooth Rolling')
xlabel('Timestep \mu s')
ylabel('Acceleration (m/s^2)')