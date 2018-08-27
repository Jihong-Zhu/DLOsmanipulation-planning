function p = pathGenBezier(x, n, m, p0, p1)
% Using bezier curve to generate path (linear -> nonlinear)
% x: parameters that needs to be optimizied
% n: number of points along the path 
% m: orders of the bezier curve
% p0: starting point
% p1: end point
t = linspace(0, 1, n)';
p = (1 - t).^m * p0 + t.^m * p1;
for i = 1 : m - 1
    p = p + nchoosek(m, i) * t.^i .* (1 - t).^(m - i) * x(2 * i - 1 : 2 * i);
end