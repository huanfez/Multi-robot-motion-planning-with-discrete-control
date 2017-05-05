clear, clc

%% Setup

% Time
dt = .02;
T = 1000; % sec

N = T/dt; % discrete horizon

% Dynamics
A = eye(2);
B = dt*eye(2);
Q = eye(2);
R = eye(2);
x0 = [1;1]; % Initial position
goalTol = .375;%.125;

x = zeros(2,N); x(:,1) = x0;
u = zeros(2,N-1);
K = dlqr(A,B,Q,R); % u = -Kx, x(i+1) = Ax(i) + Bu(i)

% Environment
gridWidth = 10;
gridLength = 10;
gridChunk = [1 4 1 4];
start = 1;
obstacles = [13, 56, 86, 88, 77, 24, 35, 45];
goals = [25, 87, 54];
goalsReached = [];
goalsLeft = goals;
ltlspec = ['LTLSPEC ! (( F (x.state = ',num2str(goals(1)),')'];%,' & F (x.state = 87) & F(x.state = 54)) )'];
for i = 2:length(goals)
    ltlspec = [ltlspec, ' & F (x.state = ',num2str(goals(i)),')'];
end
ltlspec = [ltlspec, ' ))'];

% Model Checker
fileName = 'MotionTest.smv';
sensedObstacles = []; % before implementing, device does not see obstacles
% sensedObstacles = obstacles;

[xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridLength);

[xObs,yObs] = cellPath2Grid(obstacles,gridWidth, gridLength);
[xGoal,yGoal] = cellPath2Grid(goals, gridWidth, gridLength);

%% Planning
filePath = cd;
pathNuSMV = 'C:\Program Files\NuSMV\2.5.4\bin';
makeSMV_v2(fileName, gridWidth, gridLength, gridChunk, start, sensedObstacles, ltlspec);break
tic
[~,output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
toc
[xPath,yPath,cPath] = getPath(output, gridWidth, gridLength);

%% Plot inital plan

figure(1), subplot(1,2,1)
plot(xPath,yPath,'s','MarkerSize',50)
hold on
plot(xPath(1),yPath(1),'ko',xPath(end),yPath(end),'k^','MarkerSize',25)
axis equal
axis([0 gridWidth+1 0 gridLength+1])
arrow([xPath(1:end-1)',yPath(1:end-1)'],...
    [xPath(2:end)',yPath(2:end)'],10);
text(xPath+.1,yPath+.1,num2str(cPath'))
% plot(cellPath2Grid(obstacles,gridWidth, gridLength),'x','MarkerSize',12);
% plot(xObs,yObs,'xr','MarkerSize',50)
plot(xGoal, yGoal, 'dm', 'MarkerSize', 25)
plot(xSObs,ySObs,'xr','MarkerSize',50)
axis equal
axis([0 gridWidth+1 0 gridLength+1])
title('Planning'),xlabel('x'),ylabel('y')
hold off
subplot(1,2,2),title('Pathing'),xlabel('x'),ylabel('y')
% pause

%% Initialize path plot
xText = [];
yText = [];
for i = 1:gridLength
    xText = [xText, 1:gridWidth];
    yText = [yText, i*ones(1, gridWidth)];
end

% Full grid path
figure(1)
hold off
plot(xText,yText,'s','MarkerSize',50)
axis equal
axis([0 gridWidth+1 0 gridLength+1])
hold on
text(xText+.1,yText+.25,num2str([1:gridWidth*gridLength]'))
plot(xGoal, yGoal, 'dm', 'MarkerSize', 25)
plot(xObs,yObs,'xr','MarkerSize',50)
% arrow([xPath(1:end-1)',yPath(1:end-1)'],...
%     [xPath(2:end)',yPath(2:end)'],20);
plot(xPath(1),yPath(1),'ko','MarkerSize',25)
% plot(x(1,1:k),x(2,1:k),'k-','LineWidth',2)
title('Pathing'),xlabel('x'),ylabel('y')

%% Control

pointCounter = 1;
times = zeros(1,N-1);
pause
% 
% movieObj = VideoWriter('SenseTest.avi');
% open(movieObj);
% 
% frame = getframe(gcf);
% writeVideo(movieObj,frame);

for k = 1:N-1
    tic
    if abs(xPath(pointCounter)-x(1,k)) < goalTol && ...
            abs(yPath(pointCounter)-x(2,k)) < goalTol
        
        pointCounter = pointCounter + 1;
    end
    
    if pointCounter >= length(cPath);
        break
    end
    
    u(:,k) = -K*(x(:,k)-[xPath(pointCounter);yPath(pointCounter)]);
    u(:,k) = u(:,k)/norm(u(:,k)+eps);
    
    x(:,k+1) = A*x(:,k) + B*u(:,k);
    
    % Plot path
    plot(x(1,k:k+1),x(2,k:k+1),'k-','LineWidth',2)
    
    % Has a goal been reached?
    if ismember(cPath(pointCounter-1),goalsLeft)
        goalsReached = [goalsReached, cPath(pointCounter-1)];
        
        goalsLeft = goals(ismember(goals,goalsReached)==0);
        
        if isempty(goalsLeft)
            break
        end
        
        ltlspec = ['LTLSPEC ! (( F (x.state = ',num2str(goalsLeft(1)),')'];%,' & F (x.state = 87) & F(x.state = 54)) )'];
        for i = 2:length(goalsLeft)
            ltlspec = [ltlspec, ' & F (x.state = ',num2str(goalsLeft(i)),')'];
        end
        ltlspec = [ltlspec, ' ))'];
        
    end
    
    % If obstacle on path is sensed
    if ismember(cPath(pointCounter),obstacles)
        newStart = cPath(pointCounter-1);
        sensedObstacles = [sensedObstacles, cPath(pointCounter)];
        clear xPath yPath cPath output
        makeSMV_v2(fileName, gridWidth, gridLength, gridChunk, newStart, sensedObstacles, ltlspec);
        [~,output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileName]);
        [xPath,yPath,cPath] = getPath(output, gridWidth, gridLength);
        
        pointCounter = 1;
        
        [xSObs,ySObs] = cellPath2Grid(sensedObstacles,gridWidth, gridLength);
        
        % Plot new plan
        figure(1), subplot(1,2,1)
        hold off
        plot(xPath,yPath,'s','MarkerSize',50)
        hold on
        plot(xPath(1),yPath(1),'ko',xPath(end),yPath(end),'k^','MarkerSize',25)
        axis equal
        axis([0 gridWidth+1 0 gridLength+1])
        arrow([xPath(1:end-1)',yPath(1:end-1)'],...
            [xPath(2:end)',yPath(2:end)'],10);
        text(xPath+.1,yPath+.1,num2str(cPath'))
        % plot(cellPath2Grid(obstacles,gridWidth, gridLength),'x','MarkerSize',12);
        plot(xSObs,ySObs,'xr','MarkerSize',50)
        plot(xGoal, yGoal, 'dm', 'MarkerSize', 25)
        axis equal
        axis([0 gridWidth+1 0 gridLength+1])
        title('Planning'),xlabel('x'),ylabel('y')
        hold off
        subplot(1,2,2)
        hold on
        
%         pause
        
    end
%     pause(.001)
    times(k) = toc;
    if dt-toc < 0
        fprintf('bump: %.2f\n',(toc-dt))
    else
        pause(dt-toc)
    end
%     frame = getframe(gcf);
%     writeVideo(movieObj,frame);
end

% Finish path plotting
plot(xPath(end),yPath(end),'k^','MarkerSize',25)
hold off

% frame = getframe(gcf);
% writeVideo(movieObj,frame);
% close(movieObj)