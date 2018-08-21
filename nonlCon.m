function [c, ceq] = nonlCon(x, n, m, p0, p1, cond)
% nonlinear constraints for the optimization problem
% c: inequality constraints c <= 0
% ceq: equality constraints ceq = 0
ceq = [];
p = pathGen(x, n, m, p0, p1);
xp = p(:, 1);
x = xp;
yp = p(:, 2);
d = xp.^2 + yp.^2;
d = d - 1;   % Maximum distance < total length of the DLO
for j = 1 : length(yp)
    if yp(j) < -0.2 && yp(j) > 0.2
        x(j) = cond;
    end
end
x = x - cond;
c = [d; x];