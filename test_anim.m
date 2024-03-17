clc, clear, close all;

load MPC_run.mat

num_pts = size(state_history, 2);
tf = 84;
time = linspace(0, tf, num_pts).';

bound1 = readmatrix('./global_racetrajectory_optimization/bound1.csv');

len_bound1 = size(bound1, 1);

xb1 = interp1(linspace(0, tf, len_bound1).', bound1(:, 1), time);
yb1 = interp1(linspace(0, tf, len_bound1).', bound1(:, 2), time);

bound1 = [xb1, yb1];

bound2 = readmatrix('./global_racetrajectory_optimization/bound2.csv');

len_bound2 = size(bound2, 1);

xb2 = interp1(linspace(0, tf, len_bound2).', bound2(:, 1), time);
yb2 = interp1(linspace(0, tf, len_bound2).', bound2(:, 2), time);

bound2 = [xb2, yb2];

refline= [state_history(1, :).', state_history(2, :).'];
refPathObj = referencePathFrenet(refline);
kappa = curvature(refPathObj, refPathObj.SegmentParameters(:, end));
kappa = [kappa; kappa(end)];

vx = state_history(4, :).'*1.4;
vy = state_history(5, :).';
ax = acceleration_history(4, :).';
ay = -vx .* vx .* kappa;

steering_vals = control_history(1, :).';
steering_wheel_vals = steering_vals*13;
tb_vals = rescale(control_history(2, :).', -1, 1);

videoWriterObject = VideoWriter("current_anim.mp4", 'MPEG-4');
videoWriterObject.FrameRate= round(1/mean(diff(time)));
videoWriterObject.Quality = 100;
open(videoWriterObject);

for i = 1:num_pts

    tiledlayout(2, 4, 'TileSpacing', 'tight', 'Padding', 'tight')
    set(gcf, 'Position', get(0, 'Screensize'));
    
    nexttile([2,2])


    plot(bound1(:, 1), bound1(:, 2), 'linewidth', 1.5, 'color', 'black');
    hold on;
    plot(bound2(:, 1), bound2(:, 2), 'linewidth', 1.5, 'color', 'black');
    plot(refline(:, 1), refline(:, 2), 'linewidth', 1.2, 'Color', 'blue')
    xline(refline(i, 1), 'linewidth', 1.2,'color', 'red');
    yline(refline(i, 2), 'linewidth', 1.2,'color', 'red');
    grid on

    
    text(-80, 250, "time = " + num2str(time(i), '%0.2f') + " s", 'fontsize', 20)
    
    nexttile
    
    axis off; % Turns off the axes
    axis equal; % Ensures aspect ratio is maintained
    hold on; % Keeps the plot and allows for multiple graphical objects to be added
    
    th = linspace(pi, 0); % Defines the range for a semicircle
    r = 1; % Radius of the dial
    x = r * cos(th); % x-coordinates of the dial
    y = r * sin(th); % y-coordinates of the dial
    plot(x, y, 'k', 'LineWidth', 2); % Plots the dial
    
    vmax = 250;
    delv1 = 20;
    delv2 = 10;
    for k = 0:delv1:vmax
        angle = pi * (1 - k / vmax);
        tick_x = [cos(angle), 0.9 * cos(angle)];
        tick_y = [sin(angle), 0.9 * sin(angle)];
        plot(tick_x, tick_y, 'k', 'LineWidth', 1.5); % Ticks
        text(0.8 * cos(angle), 0.8 * sin(angle), num2str(k), 'HorizontalAlignment', 'center', 'FontSize', 18); % Labels
    end

    for k = 0:delv2:vmax
        angle = pi * (1 - k / vmax);
        tick_x = [cos(angle), 0.95 * cos(angle)];
        tick_y = [sin(angle), 0.95 * sin(angle)];
        plot(tick_x, tick_y, 'k', 'LineWidth', 0.8); % Ticks
    end
    angle = pi * (1 - vx(i) * 2.23694 / vmax);
    needle = plot([0, 0.9 * cos(angle)], [0, 0.9 * sin(angle)], 'r', 'LineWidth', 2); % Initial needle position
    plot(0, 0, 'marker', 'o', 'MarkerSize', 20, 'Color','black', 'MarkerFaceColor','black')
    
    text(-0.05, 0.5, 'mph', 'FontSize', 20)

    nexttile

    theta = atan2(ax(i), ay(i));
    
    polarplot(theta, norm([ax(i); ay(i)])/9.81, 'marker', 'o', 'Color', 'blue', 'MarkerFaceColor','blue', 'MarkerSize',15);
    pax = gca;
    pax.FontSize = 20;
    pax.RAxisLocation = 45;
    rlim([0 2.0])
    thetaticks(0:30:360);
    thetaticklabels([90:-30:0, -30:-30:-150, 180:-30:120, 90]);
    rticks(0:0.4:2.0)
    rticklabels(0:0.4:2.0)
    
    nexttile

    axis off; % Turns off the axes
    axis equal; % Ensures aspect ratio is maintained
    hold on; % Keeps the plot and allows for multiple graphical objects to be added
    
    th = linspace(pi, 0); % Defines the range for a semicircle
    r = 1; % Radius of the dial
    x = r * cos(th); % x-coordinates of the dial
    y = r * sin(th); % y-coordinates of the dial
    plot(x, y, 'k', 'LineWidth', 2); % Plots the dial
    
    tmax = 1;
    delt1 = 0.2;
    delt2 = 0.1;
    for k = -tmax:delt1:tmax
        angle = pi/2 * (1 - k / tmax);
        tick_x = [cos(angle), 0.9 * cos(angle)];
        tick_y = [sin(angle), 0.9 * sin(angle)];
        plot(tick_x, tick_y, 'k', 'LineWidth', 1.5); % Ticks
        text(0.8 * cos(angle), 0.8 * sin(angle), num2str(k*100), 'HorizontalAlignment', 'center', 'FontSize', 18); % Labels
    end

    for k = -tmax:delt2:tmax
        angle = pi/2 * (1 - k / tmax);
        tick_x = [cos(angle), 0.95 * cos(angle)];
        tick_y = [sin(angle), 0.95 * sin(angle)];
        plot(tick_x, tick_y, 'k', 'LineWidth', 0.8); % Ticks
    end
    
    angle = pi/2 * (1 - tb_vals(i) / tmax);
    needle = plot([0, 0.9 * cos(angle)], [0, 0.9 * sin(angle)], 'r', 'LineWidth', 2); % Initial needle position
    
    plot(0, 0, 'marker', 'o', 'MarkerSize', 20, 'Color','black', 'MarkerFaceColor','black')
    
    text(-0.1, 0.5, 'power %', 'FontSize',20)

    nexttile
    
    steeringWheelImage = imread('images/steering_wheel.png');
    % Display the rotated image
    rotatedImage = imrotate(steeringWheelImage, rad2deg(steering_wheel_vals(i)), 'crop');
    rotatedImage(rotatedImage==0) = 255;
    imshow(rotatedImage);
   
    angle_t = "tires = " + num2str(-rad2deg(steering_vals(i)), '%0.2f') + " deg.";
    angle_s = "steering = " + num2str(-rad2deg(steering_wheel_vals(i)), '%0.2f') + " deg.";

    text(10, 10, angle_t, 'fontsize', 20)
    text(300, 10, angle_s, 'fontsize', 20)
    

    currentFrame = getframe(gcf);
    writeVideo(videoWriterObject, currentFrame);
    
    clf;
end

close(videoWriterObject);