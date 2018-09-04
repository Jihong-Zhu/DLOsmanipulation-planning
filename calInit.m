function X0 = calInit(x, n, m, p0, p1, cond)
% Generate function to calculate a feasible solution under constraints for
% the alg. to start
p = pathGen(x, n, m, p0, p1);
xp = p(:, 1);
yp = p(:, 2);
d = xp.^2 + yp.^2;
d = d - cond;   % one possible solution to the problem
X0 = d;