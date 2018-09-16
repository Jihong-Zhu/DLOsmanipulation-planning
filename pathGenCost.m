function dmin_neg = pathGenCost(x, n, m, p0, p1, refP, costtype)
% Maximize the shortest distance = minimize the negative shortest distance
% x: parameters that needs to be optimizied
% n: number of points along the path 
% m: orders of the basic function approx the path
% p0: starting point 1 * 2 vector
% p1: end point 1 * 2 vector
% refP: reference point to calculate the distance (in single arm case it is
% set to (0, 0) 1 * 2 vector
% costtype: 1 for maximum energy
%           2 for averge energy along the path 
switch nargin
    case 6
        costtype = 1;
end
p = pathGen(x, n, m, p0, p1);
% p = pathGenBezier(x, n, m, p0, p1);
p = p - refP;
d = p(:, 1).^2 + p(:, 2).^2;
switch costtype
    case 1 
        dmin_neg = - min(d);
    case 2
        dmin_neg = - sum(d);
end
% dmin_neg = -d;  % for minmax optimzation