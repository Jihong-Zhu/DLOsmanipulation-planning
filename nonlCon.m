function [c, ceq] = nonlCon(x, n, m, p0, p1, cond)
% nonlinear constraints for the optimization problem
% c: inequality constraints c <= 0
% ceq: equality constraints ceq = 0
ceq = [];
p = pathGen(x, n, m, p0, p1);
xp = p(:, 1);
yp = p(:, 2);
d = xp.^2 + yp.^2;
d = d - 1;   % Maximum distance < total length of the DLO
%---------------------Solve for varied number of constraints---------------
x = xp;
for j = 1 : length(yp)
    if yp(j) < -0.2 && yp(j) > 0.2
        x(j) = cond;
    end
end
x = x - cond;
%--------------------------------------------------------------------------
%----------------------Not working-----------------------------------------
% x = [];
% for j = 1 : length(yp)
%     if yp(j) >= -0.2 && yp(j) <= 0.2
%         x = [x; xp(j)];
%     end
% end
%--------------------------------------------------------------------------
c = [d; x];