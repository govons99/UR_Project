function eta = comp_eta(fval, x0, k, q)

opt = optimset('Display', 'off');

N = max(size(q));
eta = zeros(N, 1);

for i = 1:N
    %fun = @(e) abs(fval(i) - k*((e-q(i)) + (e-q(i))^3));
    fun = @(e) fval(i) - k*((e-q(i)) + (e-q(i))^3);
    %eta(i) = fminunc(fun, x0, opt);
    eta(i) = fsolve(fun, x0, opt);
end

end

