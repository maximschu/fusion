%% LIDAR 2-D PLOT ERROR VS TRUE DISTANCE

true_dist = sqrt(true_x.^2+true_y.^2);
lidar_error_dist = abs(true_dist-lidar_dist);
lidar_error_percent = (lidar_error_dist./true_dist).*100;

middle = lidar_error_percent(1:10);
right = lidar_error_percent(11:18);
left = lidar_error_percent(19:26);

plot(true_dist(19:26), left, 'x', 'Color','k')
hold on;
plot(true_dist(1:10), middle, '*')

plot(true_dist(11:18), right, 'o', 'Color','b')

xlabel("True Distance (m)", "FontWeight","bold")
ylabel("Error (%)", "FontWeight","bold")
grid on;
title("LIDAR: TRUE DISTANCE VS PERCENTAGE ERROR")
legend("x = -0.5", "x = 0", "x = 0.5")

%% LIDAR 3D PLOT ERROR VS TRUE-X VS TRUE-Y

lidar_error_dist = abs(true_dist-lidar_dist);
lidar_error_percent = (lidar_error_dist./true_dist).*100;

scatter3(true_x,true_y, lidar_error_percent, 40,"red","filled","o")

xlabel("x","FontWeight","bold")
ylabel("y","FontWeight","bold")
zlabel("Error", "FontWeight","bold")
title("LIDAR: X VS Y VS ERROR")

%% CAMERA ERROR VS TRUE DISTANCE

true_dist = sqrt(true_x.^2+true_y.^2);

camera_error_dist = abs(true_dist-camera_dist);
camera_error_percent = (camera_error_dist./true_dist).*100;

plot(true_dist, camera_error_percent, '*')

xlabel("True Distance (m)", "FontWeight","bold")
ylabel("Error (%)", "FontWeight","bold")
grid on;
title("CAMERA: TRUE DISTANCE VS PERCENTAGE ERROR")