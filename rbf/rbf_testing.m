clc, clear;

t0 = 0;
tf = 3;
h = 1/100;
tgrid = (t0:h:tf-h).';

load("init_control.mat", "U_opt");

U_opt = U_opt.';
% xvals = (0:0.01:1).';
% yvals = sin(2*pi*xvals);

eps = 1/mean(diff(tgrid));
params = train_rbf(tgrid, U_opt, eps);

fplot(@(x) get_control(params, x, tgrid, eps), [t0 tf], 'LineWidth', 1.0);
hold on;
plot(tgrid, U_opt, '--', 'LineWidth', 1.5);