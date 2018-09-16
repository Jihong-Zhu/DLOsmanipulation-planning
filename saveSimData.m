% To save all the relevant data for the simulation
% cond: constraint on x axis
% ymax, ymin: the value of y when x has constraints
% robPath: path of the end effector (x y angle)
% dlopath: path of the dlo (x y * 10)
clear all
close all
clc
p0 = [0.8, 0.20];
p1 = [0.75, -0.6];
cond = 0.8;
ymin = - 0.4;
ymax = 0;
theta = - pi / 6; 
numOfSamples = 10;  % number of samples along the path
numOfOrders = 3;    % number of orders 
refP = [0, 0];      % Reference point to calculate the distance, set to [0, 0] for single arm
% distance in the calculated path
f = @(x)pathGenCost(x, numOfSamples, numOfOrders, p0, p1, refP);
x0 = zeros(1, 2 * numOfOrders); 
% constraints
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
nonlcon = @(x)nonlCon(x, numOfSamples, numOfOrders, p0, p1,cond, ymin, ymax);
options = optimset('Display','iter','Algorithm','interior-point', 'MaxIter', 10000, 'MaxFunEvals', inf); %, 'OutputFcn',@plotCost);  % To check if the cost is decreasing with iteration
[x, fval] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
p = pathGen(x, numOfSamples, numOfOrders, p0, p1); 
%% Don't plan the angle
addpath('/home/jihong/DLOs_Manpiluation/dlo_modelling');
dlopath = zeros(101, 2 * numOfSamples);
n = 4; % Use 4th order approximation
init = zeros(2 * n + 2, 1);
[dlo, ~, param, ~] = dlody_Pos(0, 0, p0(1), p0(2), 0.0, init);
dlopath(:, 1:2) = dlo;
% calculate the angle
[~, rotR] = rotFromData(dlo,4);
dlo = dlodynamics(0, 0, p1(1), p1(2), 0.0, theta, init);
dlopath(:, end-1:end) = dlo;
% save data for robot control
robPath = [p0(1), p0(2), rotR];
for numOfData = 2 : length(p) - 1
    [dlo, ~, param, AngleEnd] = dlody_Pos(0, 0, p(numOfData, 1), p(numOfData, 2), 0.0, param);
    dlopath(:, numOfData * 2 - 1 : numOfData * 2) = dlo;
    [~, rotR] = rotFromData(dlo,4);     
    robPath = [robPath; p(numOfData, 1), p(numOfData, 2), rotR];
    init = param;   % use previous solution as initialization
end
robPath = [robPath; p1(1), p1(2), theta];    % data to save
save('Ex3.mat', 'cond', 'ymin', 'ymax', 'dlopath', 'robPath');
%% Plan the angle
dlopath = zeros(101, 2 * numOfSamples);
addpath('/home/jihong/DLOs_Manpiluation/dlo_modelling');
n = 4; % Use 4th order approximation
init = zeros(2 * n + 2, 1);
[dlo, ~, init, thetaEnd] = dlody_Pos(0, 0, p0(1), p0(2), 0.0, init);
dlopath(:, 1:2) = dlo;
% calculate the angle
% Plan the angle:
pRot = linspace(thetaEnd, theta, numOfSamples);
[dlo, ~] = dlodynamics(0, 0, p1(1), p1(2), 0.0, theta);
dlopath(:, end-1:end) = dlo;
for numOfData = 2 : length(p) - 1
    [dlo, param] = dlodynamics(0, 0, p(numOfData, 1), p(numOfData, 2), 0.0, pRot(numOfData), init);
    dlopath(:, numOfData * 2 - 1 : numOfData * 2) = dlo;
    init = param;   % use previous solution as initialization
end
robPath = [p pRot'];
save('Ex4.mat', 'cond', 'ymin', 'ymax', 'dlopath', 'robPath');
%% Scaling
scaleFactor = 0.4;
cond = cond * scaleFactor;
ymin = ymin * scaleFactor;
ymax = ymax * scaleFactor;
dlopath = dlopath * scaleFactor;
robPath(:, 1:2) = robPath(:, 1:2) * scaleFactor;    % angle is the same;
save('Ex4Scaled.mat', 'cond', 'ymin', 'ymax', 'dlopath', 'robPath');
%% Path save as csv
% convert to degree:
robPath(:, 3) = - robPath(:, 3) * 180 / pi;
csvwrite('Ex4.csv', robPath);