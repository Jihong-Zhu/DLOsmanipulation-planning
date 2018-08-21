function p = pathGen(x, n, m, p0, p1)
% Generate path p
% x: parameters that needs to be optimizied
% n: number of points along the path 
% m: orders of the basic function approx the path
% p0: starting point
% p1: end point
k = linspace(0, 1, n)';
p = (1 - k) * p0 + k * p1;
for i = 1 : m
    p = p + k.^i .* (1 - k) * x(2 * i - 1 : 2 * i);
end