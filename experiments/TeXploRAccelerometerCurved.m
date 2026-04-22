%Read in Vicon accelerometer data for TeXploR impact

opts = detectImportOptions('Test 5.csv');
M = readmatrix('Test 5.csv','Range','A2:H15764');

%Read arc 1 data:
time = M(1:15760, 1);
accel_x = M(1:15760, 6);
accel_y = M(1:15760, 7);
accel_z = M(1:15760, 8);
x = zeros(15760);
for i = 1:15760
    x(i) = i;
end



% Initial plotting
figure
for j = 1:15:4500
    plot(x(j), accel_x(j), 'b+', 'MarkerSize', 15, 'LineWidth', 3)
    plot(x(j), accel_y(j), 'r+', 'MarkerSize', 15, 'LineWidth', 3)
    plot(x(j), accel_z(j), 'g+', 'MarkerSize', 15, 'LineWidth', 3)
    hold on
end

ylim([-15 15])
legend('X', 'Y', 'Z')
title('Accelerometer Data for Shifting Mass 1 During Rolling Impact')
xlabel('Frequency (Hz)')
ylabel('Acceleration (m/s^2)')