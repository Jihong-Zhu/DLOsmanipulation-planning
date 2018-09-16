% Initialization of data for single arm manipulation
% p0: Original position of the cable
% p1: Target position
% theta: Target angle
% cond: constraints on x
% ymin: y constraint to fulfil cond
% ymax: y constraint to fulfil cond
%% Simulation set 1 (Rectangle)
p0 = [0.9, -0.40];
p1 = [0.75, 0.60];
cond = 0.8;
ymin = -0.2;
ymax = 0.2;
theta = 0;
%% Simulation set 2
p0 = [0.5, 0.4];
p1 = [0.8, -0.4];
cond = 0.70;
ymin = -0.1;
ymax = 0.3;
theta = - pi / 6; 
%% Simulation set 3 threshold 0.15
p0 = [0.9, -0.40];
p1 = [0.75, 0.60];
cond = 0.95;
ymin = -0.2;
ymax = 0.2;
theta = 0;
%% Simulation set 4 threshold 0.15
p0 = [0.5, 0.8];
p1 = [0.8, -0.4];
cond = 0.85;
ymin = 0.0;
ymax = 0.4;
theta = 0; 
%% Simulation set 5 threshold 0.15 for Fourier servoing
p0 = [0.9, -0.40];
p1 = [0.85, 0.4];
cond = 0.95;
ymin = -0.2;
ymax = 0.2;
theta = - pi / 5;
%% Simulation set threshold 0.2
p0 = [0.8, 0.20];
p1 = [0.75, -0.6];
cond = 0.8;
ymin = - 0.4;
ymax = 0;
theta = - pi / 6; 