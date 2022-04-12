function thetad = comp_thetad(q, k, x0, fval)

opt = optimset('Display', 'off');

N = max(size(q));
thetad = zeros(N, 1);

for i = 1:N
    fun = @(e) fval(i) - k*((e-q(i)) + (e-q(i))^3);
    %eta(i) = fminunc(fun, x0, opt);
    thetad(i) = fsolve(fun, x0, opt);
end

end

