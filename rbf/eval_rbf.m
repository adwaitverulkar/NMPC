function u = eval_rbf(p, t, tgrid, eps, basis)
    r = abs(t-tgrid);
    switch basis
        case 'linear'
            u = dot(r, p);
        case 'cubic'
            u = dot(r.^3, p);
        case 'gaussian'
            u = dot(gaussian(r, eps), p);
        case 'invmq'
            u = dot(invmq(r, eps), p);
        case 'tps'
            u = dot(r.*log(r.^r), p);
    end
        
end