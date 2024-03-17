function psi = gaussian(x, eps)
    psi = exp(-eps*eps*x.*x);
end