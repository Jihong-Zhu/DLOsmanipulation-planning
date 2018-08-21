function dmin_neg = pathGenCost(x, n, m, p0, p1)
% Maximize the shortest distance = minimize the negative shortest distance
% x: parameters that needs to be optimizied
% n: number of points along the path 
% m: orders of the basic function approx the path
% p0: starting point
% p1: end point
p = pathGen(x, n, m, p0, p1);
d = p(:, 1).^2 + p(:, 2).^2;
dmin_neg = - min(d);