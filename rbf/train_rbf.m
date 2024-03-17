function params = train_rbf(xvals, yvals, eps, basis)
    r = abs(xvals.' - xvals);
    switch basis
        case 'linear'
            params = r\yvals;
        case 'cubic'
            params = r.^3\yvals;
        case 'gaussian'
            params = gaussian(r, eps) \ yvals;
        case 'invmq'
            params = invmq(r, eps) \ yvals;
        case 'tps'
            params = r.*log(r.^r) \ yvals;
    end
end