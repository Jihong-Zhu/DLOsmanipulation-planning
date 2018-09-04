% DLOs manipulation planning to avoid obstacles in the environment. The
% planning is on a 2D plane. Single arm is used
% Jihong Zhu
clear all
close all
clc
p0 = [0.9, -0.40];
p1 = [0.75, 0.60];
cond = 0.8;
ymin = -0.2;
ymax = 0.2;
theta = 0;
numOfSamples = 10;  % number of samples along the path
numOfOrders = 3;    % number of orders 
refP = [0, 0];      % Reference point to calculate the distance, set to [0, 0] for single arm      
%% Optimization problem: it is a minmax problem to maximize the minimum
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
%% Draw initial/target shape & obstacle
addpath('/home/jihong/DLOs_Manpiluation/dlo_modelling');
init = zeros(2 * 4 + 2, 1);
figure(10)
[dlo, ~, ~] = dlody_Pos(0, 0, p0(1), p0(2), 0.0, init);
plot(dlo(:, 1), dlo(:, 2), 'k', 'LineWidth',3);
hold on;
dlo = dlodynamics(0, 0, p1(1), p1(2), 0.0, theta, init);
plot(dlo(:, 1), dlo(:, 2), '--k', 'LineWidth',3);
rectangle('Position', [cond, ymin, 0.4, ymax - ymin], 'FaceColor',[0.5 0.5 0.5],'EdgeColor','k',...
    'LineWidth',3); % obstacle
scatter(p0(1),p0(2), 100, 'filled');
scatter(p1(1),p1(2), 100, '*');
axis equal;
% legend({'initial shape', 'target shape'}, 'FontSize',16);
set(gca, 'FontSize',18);
axis off
%% Optimization
options = optimset('Display','iter','Algorithm','interior-point', 'MaxIter', 10000, 'MaxFunEvals', inf); %, 'OutputFcn',@plotCost);  % To check if the cost is decreasing with iteration
[x, fval] = fmincon(f,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
% [x, fval] = fminimax(f,x0,A,b,Aeq,beq,lb,ub,nonlcon, options);   % use minmax instead
% hold off;
%% Draw path
figure(2)
rectangle('Position', [cond, ymin, 0.4, ymax - ymin], 'FaceColor',[0.5 0.5 0.5],'EdgeColor','k',...
    'LineWidth',3); % obstacle
axis equal;
hold on;
p = pathGen(x, numOfSamples, numOfOrders, p0, p1);
scatter(p(:, 1), p(:, 2));
hold off;
%% Generate shape for each point (Andre's 3D model)
figure(3)
addpath('/home/jihong/DLOs_Manpiluation/DLO_3D');
addpath('/home/jihong/DLOs_Manpiluation/DLO_3D/Tools');
Rf=1;       % Flexural coefficient
Rt=0;       % Torsional coefficient
Re=0.0;     % extension coefficient
D=0.0;      % weight par m
L = 1;      % total length of the DLO

% Numbers of function in the series
kmax=2;         % Use 2nd order approximation
n=2*kmax+2;     % number of parameters per varaible

% Discretization
N=50;
s0=0;
s1=L;      
ds=(s1-s0)/N;

state0=zeros(1,6);  % One(fixed) end
figure(3);
for numOfData = 1 : length(p)
    state1 = [p(numOfData, 2), 0.0, p(numOfData, 1), 0.0, 0.0, 0.0]; % x -> y z -> x
    param = dlody3D(state0, state1);
    [dlo, ~] = dloData(param, n, s0, s1, ds, state0, Re);
    scatter(dlo(3, :), dlo(1, :));
    hold on;
end
axis equal;
hold off;
%% Generate shape for each point (My 2D model)
figure(4)
addpath('/home/jihong/DLOs_Manpiluation/dlo_modelling');
rectangle('Position', [cond, ymin, 0.4, ymax - ymin], 'FaceColor',[0.5 0.5 0.5],'EdgeColor','k',...
    'LineWidth',3); % obstacle
hold on;
scatter(p0(1),p0(2), 100, 'filled');
scatter(p1(1),p1(2), 100, '*');
n = 4; % Use 4th order approximation
init = zeros(2 * n + 2, 1);
[dlo, ~, ~] = dlody_Pos(0, 0, p0(1), p0(2), 0.0, init);
% calculate the angle
[~, rotR] = rotFromData(dlo,4);
plot(dlo(:, 1), dlo(:, 2), 'k', 'LineWidth',3);
dlo = dlodynamics(0, 0, p1(1), p1(2), 0.0, theta, init);
plot(dlo(:, 1), dlo(:, 2), '--k', 'LineWidth',3);
% save data for robot control
robPath = [p0(1), p0(2), rotR];
for numOfData = 2 : length(p) - 1
    [dlo, ~, param] = dlody_Pos(0, 0, p(numOfData, 1), p(numOfData, 2), 0.0, init);
    [~, rotR] = rotFromData(dlo,4);     % save data for robot control
    robPath = [robPath; p(numOfData, 1), p(numOfData, 2), rotR]; % save data for robot control
    init = param;   % use previous solution as initialization
    plot(dlo(:, 1), dlo(:, 2),'-.k', 'LineWidth',3);
end
robPath = [robPath; p1(1), p1(2), theta];   % save data for robot control
robPath(:,3) = - robPath(:,3) / pi * 180;    % save data for robot control
axis equal;
axis off;
%% Control + Planning full (My 2D model
addpath('/home/jihong/DLOs_Manpiluation/dlo_modelling');
figure(6)
n = 4; % Use 4th order approximation
init = zeros(2 * n + 2, 1);
[dloCurrent, ~, param] = dlody_Pos(0, 0, p0(1), p0(2), 0.0, init);
lambdaTranl = 0.05;
lambdaRot = 0.05;
threshold = 0.1;
rectangle('Position', [cond, ymin, 0.4, ymax - ymin], 'FaceColor',[0.5 0.5 0.5],'EdgeColor','k',...
    'LineWidth',3); % obstacle
hold on;
for numOfData = 2 : length(p)
    [dloTarget, ~, param] = dlody_Pos(0, 0, p(numOfData, 1), p(numOfData, 2), 0.0, param);
    dloInterm = dloCtrl(dloCurrent, dloTarget, lambdaTranl, lambdaRot, threshold, param);
    dloCurrent = dloInterm; % update the current
end