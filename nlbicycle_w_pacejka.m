clc, clear, close all;
addpath casadi-3.6.4-windows64-matlab2018b\
addpath rbf\
import casadi.*

M = 600;
W = M*9.81;
Iz = 1000;
lf = 1.3;
lr = 1.2;

L = lf+lr;

h = 0.25;
eps = 1e-4;

acc_sl = 20000;
brake_sl = 10000;

slips = linspace(0, 1, 20).';

Bf = 12;
Cf = 1.5;
Df = 1.2;
Ef = 0.3;

[max_f, ind_f] = max(pacejka(1, Bf, Cf, Df, Ef, slips));
Caf = (max_f/slips(ind_f));
Kaf = W*lr/L/Caf;

Br = 12;
Cr = 1.5;
Dr = 1.3;
Er = 0.3;

[max_r, ind_r] = max(pacejka(1, Br, Cr, Dr, Er, slips));
Car = max_r/slips(ind_r);
Kar = W*lf/L/Car;

K = Kaf - Kar;
fprintf("Understeer gradient = %0.2f. Kaf/Kar = %0.2f afmax = %0.2f, armax = %0.2f\n", K, Kaf/Kar, rad2deg(slips(ind_f)), rad2deg(slips(ind_r)));

fs = 50;
Ts = 1/fs;
N = 20;

ns = 8;
nu = 2;

Q = 125*diag([1, 1, 1, 0.3, 0, 0, 0, 0]);
QN = Q;
R = diag([2, 1]);

QCell = repmat({Q}, 1, N);
BigQ = blkdiag(QCell{:});

RCell = repmat({R}, 1, N);
BigR = blkdiag(RCell{:});

y = SX.sym('y', ns);
u = SX.sym('u', nu);


thrust_fcn = fit_engine_data("gaussian", 5);

% speeds = linspace(5, 110, 59);
% throttles = linspace(-1, 1, 59);
% 
% [speeds, throttles] = meshgrid(speeds, throttles);
% thrusts = full(thrust_fcn(speeds, throttles));
% 
% contourf(speeds, throttles, thrusts, 50);
% colorbar;

% Fx = thrust_fcn(y(4), y(8));
Fx = acc_sl*y(8) - W*0.03 - 0.5*1.2*1*0.3*y(4)*y(4);

w_tr = Fx*h/L;

Fzf = M*9.81*lr/L - w_tr;
Fzr = M*9.81*lf/L + w_tr;

alphaf = y(7) - atan(y(5) + lf*y(6)/y(4));
alphar = -atan(y(5) - lr*y(6)/y(4));

Ffy = pacejka(Fzf, Bf, Cf, Df, Ef, alphaf);
Fry = pacejka(Fzr, Br, Cr, Dr, Er, alphar);

ode_rhs = [y(4)*cos(y(3)) - y(5)*sin(y(3));
           y(4)*sin(y(3)) + y(5)*cos(y(3));
           y(6);
           1/M*(Fx-Ffy*sin(y(7))+M*y(5)*y(6));
           1/M*(Fry+Ffy*cos(y(7))-M*y(4)*y(6));
           1/Iz*(Ffy*cos(y(7))*lf-Fry*lr);
           u(1);
           u(2)];

f_ode = Function('f', {y, u}, {ode_rhs}, {'y', 'u'}, {'f'});

k1 = f_ode(y, u);
k2 = f_ode(y+Ts*k1/2.0, u); 
k3 = f_ode(y+Ts*k2/2.0, u);
k4 = f_ode(y+Ts*k3, u);

y_next = y + Ts*(k1 + 2*k2 + 2*k3 + k4)/6;

fd_ode = Function('fd', {y, u}, {y_next}, {'y', 'u'}, {'y_next'});

Y = SX.sym('Y', ns, N+1);
U = SX.sym('U', nu, N);
Y_next = fd_ode(Y(:, 1:end-1), U);

Yr = SX.sym('Yr', ns, N);
Y0 = SX.sym('Y0', ns);

nlp = struct;
nlp.x = [vec(Y); vec(U)];
nlp.f = bilin(BigQ, vec(Y(:, 2:end) - Yr)) + bilin(BigR, vec(U)) + bilin(QN, Y(:, end) - Yr(:, end));
nlp.p = [Y0; vec(Yr)];
nlp.g = [Y(:, 1) - Y0; 
         vec(Y(:, 2:end))-vec(Y_next)];

ymax = [Inf, Inf, Inf, 100, Inf, Inf, deg2rad(35), 1].';
ymin = [-Inf, -Inf, -Inf, 1, -Inf, -Inf, -deg2rad(35), -1].';

Ymax = repmat(ymax, 1, N+1);
Ymin = repmat(ymin, 1, N+1);

umax = [deg2rad(35)/4*3, 2].';
umin = -umax;

Umax = repmat(umax, 1, N);
Umin = repmat(umin, 1, N);

ubx = [reshape(Ymax, [], 1); reshape(Umax, [], 1)];
lbx = [reshape(Ymin, [], 1); reshape(Umin, [], 1)];

ubg = zeros((N+1)*ns,1);
lbg = ubg;

ipopt_opts = struct;
ipopt_opts.ipopt.print_level = 5;
ipopt_opts.ipopt.tol = 1e-2;
ipopt_opts.ipopt.acceptable_tol = 1e-2;
ipopt_opts.ipopt.linear_solver = 'ma57';
% ipopt_opts.ipopt.hessian_approximation = 'limited-memory';
ipopt_opts.ipopt.max_iter = 100;

solver = nlpsol('solver', 'ipopt', nlp, ipopt_opts);

ref_traj = berlin_2018(Ts);

bound1 = readmatrix('./global_racetrajectory_optimization/bound1.csv');
bound2 = readmatrix('./global_racetrajectory_optimization/bound2.csv');
refline = readmatrix('./global_racetrajectory_optimization/refline.csv');
% 
% plot(refline(:, 1), refline(:, 2), 'Color','blue');
% hold on;
% plot(ref_traj(1, :), ref_traj(2, :), 'Color','green');
% plot(bound1(:, 1), bound1(:, 2), 'Color','magenta');
% plot(bound2(:, 1), bound2(:, 2), 'Color','black');

Y0_num = ref_traj(:, 1);

num_pts = size(ref_traj, 2);

sol = solver('x0', zeros(2*N+ns*(N+1), 1), ...
      'p', [Y0_num; reshape(ref_traj(:, 1:N), [], 1)], ...
      'ubx', ubx, 'lbx', lbx, 'ubg', ubg, 'lbg', lbg);

sol = full(sol.x);

Yopt = reshape(sol(1:(N+1)*ns), ns, []);
Uopt = reshape(sol((N+1)*ns+1:end), nu, []);

i = 1;
j = 1;

while i+N-1 < num_pts

    state_history(:, j) = Y0_num;

    Yr_num = ref_traj(:, i:i+N-1);

    sol = solver('x0', sol, ...
          'p', [Y0_num; reshape(Yr_num, [], 1)], ...
          'ubx', ubx, 'lbx', lbx, 'ubg', ubg, 'lbg', lbg);

    sol = full(sol.x);

    Yopt = reshape(sol(1:(N+1)*ns), ns, []);
    Uopt = reshape(sol((N+1)*ns+1:end), nu, []);

    control_history(:, j) = [Yopt(7:8, 1); Uopt(1:2, 1)];

    Y0_num = full(fd_ode(Y0_num, Uopt(1:2)));

    i = find_closest_index(Y0_num(1), Y0_num(2), ref_traj(1:2, :).')

    j = j + 1;

end

acceleration_history = full(f_ode(state_history, control_history(3:4, :)));

numpts = size(control_history, 2);

figure
plot(state_history(1,:), state_history(2,:), 'linewidth', 3)
xlabel("X (m)");
ylabel("Y (m)");
hold on
plot(ref_traj(1, 1:end-N), ref_traj(2, 1:end-N), '--', 'linewidth', 1.5)

figure
plot(linspace(0, Ts*(numpts-1), numpts), state_history(4,:), 'linewidth', 3)
xlabel("time (s)");
ylabel("v_x (m/s)");

figure
plot(linspace(0, Ts*(numpts-1), numpts), control_history(1, :))
xlabel("time (s)")
ylabel("steering (rad)")

figure
plot(linspace(0, Ts*(numpts-1), numpts), control_history(2, :))
xlabel("time (s)")
ylabel("throttle-brake")

figure
plot(linspace(0, Ts*(numpts-1), numpts), control_history(3, :))
xlabel("time (s)")
ylabel("steering rate (rad/sec)")

figure
plot(linspace(0, Ts*(numpts-1), numpts), control_history(4, :))
xlabel("time (s)")
ylabel("throttle-brake rate")

% save("MPC_run.mat", 'Ts', 'state_history', 'control_history', 'acceleration_history')




