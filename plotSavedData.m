% plot saved data
% This script plots the data saved from saveSimData.m
% load Ex1scaled.mat
% rectangle('Position', [cond, ymin, 0.4, ymax - ymin], 'FaceColor',[0.5 0.5 0.5],'EdgeColor','k',...
%     'LineWidth',3); % obstacle
% hold on;
scatter(robPath(1, 1), robPath(1, 2), 200, 'filled');
hold on;
scatter(robPath(end, 1), robPath(end, 2), 200, '*');
plot(dlopath(:, 1), dlopath(:, 2), 'k', 'LineWidth',3);
plot(dlopath(:, end -1), dlopath(:, end), '--k', 'LineWidth',3);
for numOfData = 2 : length(robPath) - 1
    plot(dlopath(:, numOfData * 2 - 1), dlopath(:, numOfData * 2),'-.k', 'LineWidth',3);
end
axis equal
% axis off
