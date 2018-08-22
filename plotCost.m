function stop = plotCost(x, optimValues, state)
% Output function to save the cost after each iteration
stop = false;
hold on;
plot(optimValues.iteration, optimValues.fval, '*');
drawnow;