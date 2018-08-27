% DLOs manipulation planning to avoid obstacles in the environment. The
% planning is on a 2D plane. Single arm is used
% Jihong Zhu
% starting point 
p0 = [0.9, -0.40];
% p0 = [0, 0];
% end point 
p1 = [0.75, 0.60];
% p1 = [0.1, 0.6];
%% Optimization problem: it is a minmax problem to maximize the minimum
% distance in the calculated path
numOfSamples = 10;  % number of samples along the path
numOfOrders = 3;    % number of orders 
refP = [0, 0];      % Reference point to calculate the distance, set to [0, 0] for single arm      
f = @(x)pathGenCost(x, numOfSamples, numOfOrders, p0, p1, refP);
x0 = zeros(1, 2 * numOfOrders); 
% constraints
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
cond = 0.8;
nonlcon = @(x)nonlCon(x, numOfSamples, numOfOrders, p0, p1,cond);
%% Optimization
% options = optimset('Display','iter');
options = optimset('OutputFcn',@plotCost);  % To check if the cost is decreasing with iteration
[x,fval,exitflag,output,lambda,grad,hessian] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
hold off;
% [x,fval] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,nonlcon);
%% Draw path
figure(2)
rectangle('Position', [0.6, -0.2, 0.4, 0.4]); % obstacle
axis([0 1 -1 1])
axis equal;
hold on;
p = pathGen(x, numOfSamples, numOfOrders, p0, p1);
plot(p(:, 1), p(:, 2));
hold off;