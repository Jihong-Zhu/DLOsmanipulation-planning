function [c, ceq] = nonlConCircle(x, n, m, p0, p1, obx, oby, radius)
% nonlinear constraints for the optimization problem (Assume a circular object)
% c: inequality constraints c <= 0
% ceq: equality constraints ceq = 0
% x: parameters that needs to be optimizied
% n: number of points along the path 
% m: orders of the basic function approx the path
% p0: starting point 1 * 2 vector
% p1: end point 1 * 2 vector
% obx: x poistion of the circle
% oby: y position of the circle
% radius: radius of the circle
ceq = [];
p = pathGen(x, n, m, p0, p1);
% p = pathGenBezier(x, n, m, p0, p1);
xp = p(:, 1);
yp = p(:, 2);
d = xp.^2 + yp.^2;
d = d - 1;   % Maximum distance < total length of the DLO
% obstacle 
dToObs = (obx - xp).^2 + (oby - yp).^2; % distance to the obstacle
ob = radius^2 - dToObs; % The point is outside of the circle 
% c = [d; x; -xp]; % For IJRR result
c = [d; ob];