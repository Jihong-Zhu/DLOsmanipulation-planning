% DLOs manipulation planning to avoid obstacles in the environment. The
% planning is on a 2D plane. 
% Jihong Zhu
% start point 
p0 = [0.9, -0.4];
% end point 
p1 = [0.75, 0.60];
% Obstacles
rectangle('Position', [0.8, -0.2, 0.4, 0.4]);
axis([0 1 -1 1])
axis equal;
hold on;
% Optimization problem: it is a minmax problem to maximize the minimum
% distance in the calculated path
numOfSamples = 10; % number of samples along the path
numOfOrders = 3;    % number of orders 
f = @(x)pathGenCost(x, numOfSamples, numOfOrders, p0, p1);
x0 = zeros(1, 2 * numOfOrders); % initialization
% constraints
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
cond = 0.8;
nonlcon = @(x)nonlCon(x, numOfSamples, numOfOrders, p0, p1,cond);
% Optimization
[x,fval] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,nonlcon);
p = pathGen(x, numOfSamples, numOfOrders, p0, p1);
plot(p(:, 1), p(:, 2));