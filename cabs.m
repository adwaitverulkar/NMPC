function x_abs = cabs(x)
    eps = 1e-3;
    x_abs = sqrt(x*x+eps);
end