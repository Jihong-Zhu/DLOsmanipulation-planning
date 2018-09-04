% Generate path and shape planning 
%% Using DLO model
% addpath('/home/jihong/DLOs_Manpiluation/dlo_modelling'); % add the DLO model
addpath('/home/jihong/DLOs_Manpiluation/DLO_3D');
addpath('/home/jihong/DLOs_Manpiluation/DLO_3D/Tools')
%% Plot the path
figure(1);
load pathSingleArm.mat
rectangle('Position', [0.8, -0.2, 0.4, 0.4]); % obstacle
% ObsVector = [0.8, -0.2; 0.8, 0.2];
% plot(ObsVector(:, 1), ObsVector(:, 2));
axis([0 1 -1 1])
axis equal;
hold on;
scatter(p(1, 1), p(1, 2), 'o'); % starting point
scatter(p(end, 1), p(end, 2), '*'); % end point
plot(p(:, 1), p(:, 2), '-o');
hold off;
%% Generate DLO along the path
figure(2)
hold on;
axis([-1 1 -1 1])
axis equal;
state0 = zeros(1, 6);
% for k = 1 : numOfSamples
    state1 = state0;
    state1(1) = p(1, 1);
    state1(3) = p(1, 2);
    param = dlody3D(state0, state1);
    [d, ~] = dloData(param, 6, 0, 1, 0.1, state0, 0.0);
    sz = 25;
    scatter(d(1, :), d(3, :), sz, 'filled');
% end
