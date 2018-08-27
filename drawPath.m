% Generate path and shape planning 
%% Using DLO model
addpath('/home/jihong/DLOs_Manpiluation/dlo_modelling'); % add the DLO model
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
axis([0 1 -1 1])
axis equal;
% for k = 1 : 3
%     if (k ~=4 && k~=6)
dloShape = dlody_Pos(0, 0, p(1, 1), p(1, 2));
sz = 25;
scatter(dloShape(:, 1), dloShape(:, 2), sz, 'filled');
%     end
% end
