%% Plot the environmnet
clear,clc

gridWidth = 10;
gridLength = 10;

start = 1;
% obstacles = [13, 56, 86, 88, 77, 24, 35, 45];
% goals = [25, 87, 54];
obstacles = [13, 56, 86, 88, 77, 24, 35, 45, 83, 94, 80, 70, 69, 48, 38, 95];
goals = [25, 54, 84, 100, 17, 87];% agent per row
sensedObstacles = obstacles;

[xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridLength);
[xObs,yObs] = cellPath2Grid(obstacles,gridWidth, gridLength);
[xGoal,yGoal] = cellPath2Grid(goals, gridWidth, gridLength);

xText = [];
yText = [];
for i = 1:gridLength
    xText = [xText, 1:gridWidth];
    yText = [yText, i*ones(1, gridWidth)];
end

xGrid = [1:gridWidth+1, ones(1,gridLength+1); 1:gridWidth+1, (gridLength+1)*ones(1,gridLength+1)]-.5;
yGrid = [ones(1,gridWidth+1), 1:gridLength+1; (gridWidth+1)*ones(1,gridLength+1), 1:gridLength+1]-.5;

% Full grid path
figure(3)
hold off
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
hold on
text(xText+.1,yText+.25,num2str([1:gridWidth*gridLength]'))
plot(xGoal, yGoal, 'dm', 'MarkerSize', 25)
plot(xObs,yObs,'xr','MarkerSize',50)
title('Pathing'),xlabel('x'),ylabel('y')
hold off