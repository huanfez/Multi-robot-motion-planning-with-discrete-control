clear
clc;

k_f = 1;
k=1;
numOFigs = 1;

gridWidth = 10;
gridLength = 10;

% %agents and obstacles information 1
% start = [31;3;41]; % per agent
% obstacles = [13, 56, 86, 88, 77, 24, 35, 45, 83, 94, 80, 70, 69, 48, 38, 95, 82, 83];
% goals = [{[54, 36]}; {[17, 87]}; {[64, 28]}];% agent per row
% goals1Dem = [54, 36, 17, 87, 64, 28];

% %agents and obstacles information 2
% start = [5; 41; 49]; % per agent
% obstacles = [91 100];
% goals = [{75}; {48}; {42}];% agent per row
% goals1Dem = [75, 48, 42];

%agents and obstacles information 3
start = [31;41;3]; % per agent
obstacles = [13, 56, 86, 88, 77, 24, 35, 45, 94, 80, 70, 69, 48, 38, 95, 82, 83];
goals = [{[54, 36]}; {[64, 27]}; {[17, 87]}];% agent per row
goals1Dem = [54, 36, 64, 27, 17, 87];

M = 3; % number of agents used

%to be used in collision
goalsReached = cell(M,1);
goalsReachedAll = [];
goalsLeft = goals;
goalTol = .125; % "radius" of accepting box in cell

sensedObstacles = cell(M,1); % sensed obstacles begin as empty and seperate

% Conversion from cell to cartesion
xSObs = cell(M,1);
ySObs = cell(M,1);
[xObs,yObs] = cellPath2Grid(obstacles,gridWidth, gridLength);
[xGoal,yGoal] = cellPath2Grid(goals1Dem, gridWidth, gridLength);
[xStart,yStart] = cellPath2Grid(start', gridWidth, gridLength);

mR = .2; % agent radius
sR = 1; % sensor radius

% Time
dt = .02; % sec

% Dynamics
A = eye(2);
B = dt*eye(2);
Q = eye(2);
R = eye(2);
x = cell(M,1); % cell to store array for each agent
u = cell(M,1);

for m = 1:M % assign x0 to each agent
    [x0,y0] = cellPath2Grid(start(m),gridWidth,gridLength); % change cell to grid coords
    x{m}(:,1) = [x0;y0];
end

K = dlqr(A,B,Q,R);

%% Model Checker Setup

ltlspec = cell(M,1); % Initialize ltlspecs for SMV model
for m = 1:M
    ltlspec{m} = ['LTLSPEC ! (( F (x.state = ',num2str(goals{m}(1)),')'];
    for i = 2:length(goals{m}) % i here reperesenting goal.  If only one goal, loop is skipped
        ltlspec{m} = [ltlspec{m}, ' & F (x.state = ',num2str(goals{m}(i)),')'];
    end
    ltlspec{m} = [ltlspec{m}, ' ))'];
end
% file locations
fileName = 'MultiTest.smv';
fileNameS = 'MultiTestC.smv';
filePath = 'D:\livelockAutonomous3\SymCodes';

%% Plot Planning Environment
% This is the initialized plot for the planning figure

% Create gridlines.  Offset by -.5 to create boxes around points
xGrid = [1:gridWidth+1, ones(1,gridLength+1); 1:gridWidth+1, (gridLength+1)*ones(1,gridLength+1)]-.5;
yGrid = [ones(1,gridWidth+1), 1:gridLength+1; (gridWidth+1)*ones(1,gridLength+1), 1:gridLength+1]-.5;

figure(1)
if numOFigs == 1
    subplot(1,2,1) % Use if want one figure, comment out otherwise
end

hold off % To ensure figure is refreshed
plot(xGrid,yGrid,'k') % plot grid lines
axis equal % to make proportions proper
axis([0 gridWidth+1 0 gridLength + 1]);
hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12) % Plot goals as magenta diamonds
plot(xStart(1),yStart(1),'ob','MarkerSize',12,'LineWidth',2) % plot agents as black circles
plot(xStart(2),yStart(2),'og','MarkerSize',12,'LineWidth',2)
plot(xStart(3),yStart(3),'or','MarkerSize',12,'LineWidth',2)
% plot(xStart(4),yStart(4),'oy','MarkerSize',12,'LineWidth',2)
%plot start point
goalPlot(xGoal,yGoal,.4,'m');
obsPlot(xObs,yObs,.3,'k',3);
% title('Pathing')
xlabel('x'),ylabel('y')
title('Planning')
set(gca,'fontsize',20)
pause(.1) % to ensure update of plot

%% Initial Planning
xPath = cell(M,1); % path in cartesian x
yPath = cell(M,1); % path in cartesian y
cPath = cell(M,1); % path in cell
gridChunk = zeros(M,4); % smallest rectangle required to produce plan
mTrack = cell(M,1); % for collision comm, stores current and next position in plan

for m = 1:M % for each agent, m
    expansion = -1; % used to expand gridChunk if required
    while isempty(cPath{m}) % while no feasible plan exists for current gridChunk
        expansion = expansion + 1; % expand the grid
        gridChunk(m,:) = getChunk(start(m), goals{m}, gridWidth, gridLength, expansion);
        hold on % still on Figure 1, planning environment
        rectangle('Position',[gridChunk(m,1)-.5, gridChunk(m,3)-.5, ...
            gridChunk(m,2)-gridChunk(m,1)+1, gridChunk(m,4)-gridChunk(m,3)+1],...
            'LineWidth',6)
        pause(0.01) % ensure plot update, slow enough to view growth
        makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), start(m), sensedObstacles{m}, ltlspec{m});
        % update SMV model according to new chunk
        [~, output] = system(['NuXMV ' filePath '\' fileName],'-echo');
        % run model checking
        [xPath{m},yPath{m},cPath{m}] = getPath(output, gridWidth, gridLength);
        % back out path from output of model checker
    end
    mTrack{m} = cPath{m}(1:2); % update collision comm variable
end

%% Plot Initial Plan
% Still on figure 1
axis equal
axis([0 gridWidth+1 0 gridLength+1])
hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12)
%%YW: initial plan
for m = 1:M
    cEnd = find(ismember(cPath{m},goalsLeft{m}),1,'last');
    plot(xPath{m}(1),yPath{m}(1),'bo',xPath{m}(cEnd),yPath{m}(cEnd),'b^',...
        'MarkerSize',12) % Plot start and end points of path
    arrow([xPath{m}(1:cEnd-1)',yPath{m}(1:cEnd-1)'],...
        [xPath{m}(2:cEnd)',yPath{m}(2:cEnd)'],'Color','b','LineWidth',1); % plot path as arrows
    rectangle('Position',[gridChunk(m,1)-.5, gridChunk(m,3)-.5, ...
        gridChunk(m,2)-gridChunk(m,1)+1, gridChunk(m,4)-gridChunk(m,3)+1],...
        'LineWidth',6); % plot final gridChunk
end
title('Planning')
set(gca,'fontsize',20)
hold off

%% Initialize Realtime Environment
if numOFigs == 2
    figure(2); % if wanted on one figure
else
    subplot(1,2,2) % if wanted on one figure
end

hold off
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
xlabel('x'),ylabel('y')
hold on
% plot(xGoal,yGoal,'dm','MarkerSize',12)
goalPlot(xGoal,yGoal,.4,'m');

pos = zeros(M,3); % stores agent plot marker.  used for delete function to update marker
for m = 1:M
    %     pos(m,1) = plot(x{m}(1,1),x{m}(2,1),'ko','MarkerSize',12);
    pos(m,1) = circle(x{m}(1,1),x{m}(2,1),mR,'b-');
    pos(m,2) = text(x{m}(1,1)+.15,x{m}(2,1)+.25,num2str(m));
    set(pos(m,2),'fontsize',16)
    pos(m,3) = circle(x{m}(1,1),x{m}(2,1),sR,'b--');
end
title('Environment')
set(gca,'fontsize',20)
clc
fprintf('Press any key to continue\n')
pause % allow adjustment of figures before running

%% Plan Variable Initialization
pointCounter = 2*ones(M,1); % used for tracking progression in x/y/cPath
kFin = zeros(M,1); % used for tracking when agent has completed all goals
newStart = start; % used to update plan
mStopped = zeros(M,1);
mCollision = cell(M,1);
mCollision2 = zeros(M,1);% used to determine collision priority

obsDisAll = 2*ones(M,length(obstacles));
obsComplex = zeros(M,1);
hObs = cell(M,1);

%% time record start
timeStart = tic;
%%%%%%%%%%%%%%%%%%%%

goalsEmpty = 1;
while goalsEmpty ~= 0
      
    %% 1.check for obstacles
    
    % check for surrounded obstacles
    for m = 1:M
        % equivalent to observation size can sense 8 neighboring cells,
        % commmunication also 8 neighboring cells
        cSurround = [cPath{m}(pointCounter(m)-1)-1, cPath{m}(pointCounter(m)-1)+1, cPath{m}(pointCounter(m)-1)-gridWidth, cPath{m}(pointCounter(m)-1)+gridWidth];
        % if any adjacent cells contain an obstacle
        if sum(ismember(cSurround,obstacles(~ismember(obstacles,sensedObstacles{m})))) > 0
            sensedObstacles{m} = [sensedObstacles{m}, cSurround(ismember(cSurround,obstacles(~ismember(obstacles,sensedObstacles{m}))))];
            [xSObs{m},ySObs{m}] = cellPath2Grid(sensedObstacles{m},gridWidth, gridLength);
            delete(hObs{m});
            hObs{m} = obsPlot(xSObs{m},ySObs{m},.3,'k',3);
        end
    end
    
    % sensor sharing
    for m = 1:M
        for m2 = 1:M
            if m ~= m2 && sqrt((xPath{m}(pointCounter(m)-1)-xPath{m2}(pointCounter(m2)-1))^2 + ...
                    (yPath{m}(pointCounter(m)-1)-yPath{m2}(pointCounter(m2)-1))^2) - 0.1 < 2*sR
                sensedObstacles{m} = [sensedObstacles{m} sensedObstacles{m2}(~ismember(sensedObstacles{m2},sensedObstacles{m}))];
                % if new obstacle is along path, update
            end
        end
    end
    
    % replaning for detected obstacles
    for m = 1:M
        if sum(ismember(cPath{m}(pointCounter(m):length(cPath{m})),sensedObstacles{m})) > 0
            ObstacleReplanScript
        end
    end
    
    %% 2. Robot Collision detection
    
    for m = 1:M % for each agent
        if pointCounter(m)>numel(cPath{m})
            cPath{m}(pointCounter(m)) = cPath{m}(pointCounter(m)-2);
        end
        mTrack{m} = cPath{m}(pointCounter(m)-1:pointCounter(m)); % update collision tracker
    end
    Clusters = [];
    for m = 1:M
        for m2 = 1:M
            if m2 > m
                IsCollidion1 = ismember(cPath{m}(pointCounter(m)),mTrack{m2}(2));
                IsCollidion2 = ismember(mTrack{m}(2),cPath{m2}(pointCounter(m2)-1)) && cPath{m}(pointCounter(m)-1) == cPath{m2}(pointCounter(m2));
                if IsCollidion1 || IsCollidion2% collision into same place 
                    mCollision{m}(m2) = -1;
                    for neighNum = [(-gridWidth - 1):(-gridWidth + 1), -1:1, (gridWidth - 1):(gridWidth + 1)]
                        Clusters = [Clusters, cPath{m}(pointCounter(m)-1)+neighNum, cPath{m2}(pointCounter(m2)-1)+neighNum];
                    end
                else
                    mCollision{m}(m2) = 0;
                end
            end
        end
    end
    
    if ismember(-1,cell2mat(mCollision))
        LocRegion = unique(Clusters);
        % 2.2 Find local final states
        tempFinalState = zeros(M,1);
        tempNum = zeros(M,1);
        for m = 1:M
            for tempPath = cPath{m}((pointCounter(m)):end)
                tempNum(m) = tempNum(m) + 1;
                if ~ismember(tempPath,LocRegion)
                    tempFinalState(m) = tempPath;
                    break
                end
            end
        end
        % 2.3 Find local path without collision
        CollisionReplanScript
    end
    %% 3. LQR
    stepFish = zeros(M,1);
    
    while sum(stepFish) ~= M
      
        for m = 1:M
            
            if abs(xPath{m}(pointCounter(m))-x{m}(1,k)) < goalTol && ...
                    abs(yPath{m}(pointCounter(m))-x{m}(2,k)) < goalTol
                stepFish(m) = 1; % reached next cell
            end
            
            %% Dynamics
            if pointCounter(m)>numel(xPath{m}) %for final state
                xPath{m}(pointCounter(m)) = xPath{m}(pointCounter(m)-2);
            end
            if pointCounter(m)>numel(yPath{m})
                yPath{m}(pointCounter(m)) = yPath{m}(pointCounter(m)-2);
            end
            u{m}(:,k) = -K*(x{m}(:,k)-[xPath{m}(pointCounter(m));yPath{m}(pointCounter(m))]);
            % Calculate input via dlqr
            u{m}(:,k) = u{m}(:,k)/norm(u{m}(:,k)+eps); % normalize for constant speed
            x{m}(:,k+1) = A*x{m}(:,k) + B*u{m}(:,k); % update position
            
            %% Real Time Plot Update
            delete(pos(m,:)) % Delete current agent image
            %pos(m,1) = plot(x{m}(1,k+1),x{m}(2,k+1),'ko','MarkerSize',12); % update agent image
            pos(m,1) = circle(x{m}(1,k+1),x{m}(2,k+1),mR,'b-');
            pos(m,2) = text(x{m}(1,k+1)+.15,x{m}(2,k+1)+.25,num2str(m)); % update agent label
            set(pos(m,2),'fontsize',16)
            pos(m,3) = circle(x{m}(1,k+1),x{m}(2,k+1),sR,'b--');
             % ensure plot updates
        end
        k = k +1;
        pause(.0001)
    end
    
    %% Path Updating
    for m = 1:M
        % if next cell has been reached
        if abs(xPath{m}(pointCounter(m))-x{m}(1,k)) < goalTol && ...
                abs(yPath{m}(pointCounter(m))-x{m}(2,k)) < goalTol
            % if reached cell is a goal, update goal parameters
            %%%Here pointerCounter(m) means the next position has been reached, which is current position
            if ismember(cPath{m}(pointCounter(m)),goalsLeft{m})
                goalsReached{m} = [goalsReached{m}, cPath{m}(pointCounter(m))];
                goalsReachedAll = [goalsReachedAll, goalsReached{m}];
                goalsLeft{m}(goalsLeft{m} == cPath{m}(pointCounter(m))) = [];
                
                fillGoal(cPath{m}(pointCounter(m)),.4,gridWidth,gridLength,'m');
                % if this is the last goal
                if isempty(goalsLeft{m})
                    % update the mTrack to show it's not moving
                    mTrack{m} = [cPath{m}(pointCounter(m)),cPath{m}(pointCounter(m))];
                end
            end
            if ~isempty(goalsLeft{m})
                pointCounter(m) = pointCounter(m) + 1;% and then update pointCounter
            end
        end
    end
    
    if pointCounter(m)>numel(cPath{m})
        cPath{m}(pointCounter(m)) = cPath{m}(pointCounter(m)-2);
    end
    if pointCounter(m)>numel(xPath{m})
        xPath{m}(pointCounter(m)) = xPath{m}(pointCounter(m)-2);
    end
    if pointCounter(m)>numel(yPath{m})
        yPath{m}(pointCounter(m)) = yPath{m}(pointCounter(m)-2);
    end
    
    goalsEmpty = 0;
    for m = 1:M
        goalsEmpty = goalsEmpty + numel(goalsLeft{m});
    end
end

%% time record start
toc(timeStart) % display how long to run the simulation
%%%%%%%%%%%%%%%%%%%%

%% Plot Final Paths

if numOFigs == 2
    figure(2)
else
    subplot(1,2,2)
end
plot(xGrid,yGrid,'k')
axis equal
axis([0 gridWidth+1 0 gridLength+1])
% title('Final Path')
xlabel('x'),ylabel('y')
hold on
goalPlot(xGoal,yGoal,.4,'m');
for m = 1:M
    if m == 1
        plot(xStart(m),yStart(m),'bs','LineWidth',2)
        plot(x{m}(1,1:k),x{m}(2,1:k),'b-','LineWidth',3)
    elseif m == 2
        plot(xStart(m),yStart(m),'gs','LineWidth',2)
        plot(x{m}(1,1:k),x{m}(2,1:k),'g-','LineWidth',3)
    elseif m == 3
        plot(xStart(m),yStart(m),'rs','LineWidth',2)
        plot(x{m}(1,1:k),x{m}(2,1:k),'r-','LineWidth',3)
    end
end
hold off
