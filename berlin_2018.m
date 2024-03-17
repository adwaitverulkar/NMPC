function Yr = berlin_2018(Ts)

    ref_traj = readmatrix("berlin_2018.csv");
    
    ref_x = ref_traj(:, 2);
    ref_y = ref_traj(:, 3);
    ref_z = zeros(size(ref_y));

    ref_vx = ref_traj(:, end-1);
    
    ref_traj = waypointTrajectory([ref_x ref_y ref_z], GroundSpeed=ref_vx);
    ref_traj.SampleRate = 1/Ts;
    
    num_pts = round(ref_traj.TimeOfArrival(end)*ref_traj.SampleRate);
    ref_traj.SamplesPerFrame = num_pts;

    [position, orientation, velocity, ~, angularVelocity] = ref_traj();

    Yr(1:2, :) = position(1:end-1, 1:2).';
    
    eulerAngles = quat2eul(orientation, 'ZYX');
    Yr(3, :) = unwrap(eulerAngles(1:end-1, 1).');

    Yr(4, :) = vecnorm(velocity(1:end-1, :).');
    Yr(5, :) = zeros(1, num_pts-1);
    Yr(6, :) = angularVelocity(1:end-1, 3).';

    Yr(7:8, :) = zeros(2, num_pts-1);

end