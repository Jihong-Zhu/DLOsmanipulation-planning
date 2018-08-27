function [c, ceq] = nonlCon(x, n, m, p0, p1, cond)
% nonlinear constraints for the optimization problem
% c: inequality constraints c <= 0
% ceq: equality constraints ceq = 0
% x: parameters that needs to be optimizied
% n: number of points along the path 
% m: orders of the basic function approx the path
% p0: starting point 1 * 2 vector
% p1: end point 1 * 2 vector
% cond: conditions
ceq = [];
p = pathGen(x, n, m, p0, p1);
% p = pathGenBezier(x, n, m, p0, p1);
xp = p(:, 1);
yp = p(:, 2);
d = xp.^2 + yp.^2;
d = d - 1;   % Maximum distance < total length of the DLO
x = xp;
for j = 1 : length(yp)
    if yp(j) < -0.2 && yp(j) > 0.2
        x(j) = cond;
    end
end
x = x - cond;
% c = [d; x; -xp]; % For IJRR result
c = [d; x];