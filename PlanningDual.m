% DLOs manipulation planning to avoid obstacles in the environment. The
% planning is on a 2D plane. Dual arm is used
% Jihong Zhu
%% Plot the planning problem
% right starting point 
pRight0 = [0.9, -0.4];
% right end point 
pRight1 = [0.75, 0.60];
% left starting point 
pLeft0 = [0, 0];
% left end point 
pLeft1 = [0.1, 0.8];
% initial
pInit = [pRight0; pLeft0];
% target
pTarget = [pRight1; pLeft1];
%% Optimization problem
RightTraj = [pRight0];
LeftTraj = [pLeft0];
numOfSamples = 10;  % number of samples along the path
numOfOrders = 3;    % number of orders 
for i = 1 : numOfSamples
% Start planning from the right
    xr0 = zeros(1, 2 * numOfOrders);
    xl0 = zeros(1, 2 * numOfOrders);
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    cond1 = 0.8;
    cond2 = 0.1;
    % Right
    f = @(x)pathGenCost(x, numOfSamples + 1 - i, numOfOrders, pRight0, pRight1, pLeft0);
    nonlcon = @(x)nonlConDual(x, numOfSamples + 1 - i, numOfOrders, pRight0, pRight1, pLeft0, cond1, cond2);
    [x,~] = fmincon(f,xr0,A,b,Aeq,beq,lb,ub,nonlcon);
    % implement only the first point
    p = pathGen(x, numOfSamples + 1 - i, numOfOrders, pRight0, pRight1);
    pRight0 = p(2, :);  % update pRight
%     xr0 = x;    % update initial value of optimization problem
    % Left
    f = @(x)pathGenCost(x, numOfSamples + 1 - i, numOfOrders, pLeft0, pLeft1, pRight0);
    nonlcon = @(x)nonlConDual(x, numOfSamples + 1 - i, numOfOrders, pLeft0, pLeft1, pRight0, cond1, cond2);
    [x,~] = fmincon(f,xl0,A,b,Aeq,beq,lb,ub,nonlcon);
    % implement only the first point
    p = pathGen(x, numOfSamples + 1 - i, numOfOrders, pLeft0, pLeft1);
    pLeft0 = p(2, :);  % update pRight
%     xl0 = x;    % update initial value of optimization problem
    RightTraj = [RightTraj; pRight0];
    LeftTraj = [LeftTraj; pLeft0];
end
%% draw
rectangle('Position', [0.8, -0.2, 0.4, 0.4]); % obstacle 1
hold on;
rectangle('Position', [-0.1, 0.1, 0.2, 0.2]); % obstacle 2
scatter(pInit(:, 1), pInit(:, 2), 'o');
scatter(pTarget(:, 1), pTarget(:, 2), '*');
plot(RightTraj(:, 1), RightTraj(:, 2));
plot(LeftTraj(:, 1), LeftTraj(:, 2));
axis([0 1 -1 1])
axis equal;
hold off;