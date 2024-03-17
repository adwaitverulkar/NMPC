function Yr = circle_track(R, Ts, lap_time)
   
    h = 0;
    k = R;

    num_pts = lap_time/Ts;

    theta = linspace(-pi/2, 1.5*pi, num_pts).';

    x = h + R * cos(theta);
    y = k + R * sin(theta);
    z = zeros(size(x));

    TimeOfArrival = linspace(0, lap_time, num_pts).';
    trajectory = waypointTrajectory([x y z], TimeOfArrival);
    trajectory.SamplesPerFrame = num_pts;

    [position, orientation, velocity, ~, angularVelocity] = trajectory();

    Yr(1:2, :) = position(1:end-1, 1:2).';
    
    eulerAngles = quat2eul(orientation, 'ZYX');
    Yr(3, :) = unwrap(eulerAngles(1:end-1, 1).');

    Yr(4, :) = trajectory.GroundSpeed(1:end-1).';
    Yr(5, :) = zeros(1, num_pts-1);
    Yr(6, :) = angularVelocity(1:end-1, 3).';

    Yr(7:8, :) = zeros(2, num_pts-1);
end