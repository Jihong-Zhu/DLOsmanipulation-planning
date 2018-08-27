function [c, ceq] = nonlConDual(x, n, m, p0, p1, refP, cond1, cond2)
% nonlinear constraints for the optimization problem
% c: inequality constraints c <= 0
% ceq: equality constraints ceq = 0
% x: parameters that needs to be optimizied
% n: number of points along the path 
% m: orders of the basic function approx the path
% p0: starting point 1 * 2 vector
% p1: end point 1 * 2 vector
% refP: reference point to calculate the distance (in single arm case it is
% set to (0, 0) 1 * 2 vector
ceq = [];
p = pathGen(x, n, m, p0, p1);
xp = p(:, 1);
yp = p(:, 2);
% condition 1:
xCond1 = xp;
for j = 1 : length(yp)
    if yp(j) < -0.2 && yp(j) > 0.2
        xCond1(j) = cond1;
    end
end
x1 = xCond1 - cond1;
% condition 2:
xCond2 = xp;
for j = 1 : length(yp)
    if yp(j) < 0.1 && yp(j) > 0.3
        xCond2(j) = cond2;
    end
end
x2 = cond2 - xCond2;
% length condition:
pRef = p - refP;
d = pRef(:, 1).^2 + pRef(:, 2).^2;
d = d - 1;   % Maximum distance < total length of the DLO
c = [d; x1; x2];