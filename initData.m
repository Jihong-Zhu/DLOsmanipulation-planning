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
p0 = [0.5, 0.8];
p1 = [0.8, -0.4];
cond = 0.70;
ymin = 0.0;
ymax = 0.4;
theta = 0; 
%% 
