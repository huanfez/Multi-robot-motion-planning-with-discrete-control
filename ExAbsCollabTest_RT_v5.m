% Redesigned for trust based goal reassign

clear
clc;

k_f = 1;
k=1;
numOFigs = 1;

testvalue = 0;
testvalue2 =0 ;

gridWidth = 10;
gridLength = 10;

% %agents and obstacles information 1
% start = [31;3;41]; % per agent
% obstacles = [2, 13, 56, 58, 86, 88, 77, 24, 35, 45, 83, 94, 80, 70, 69, 48, 38, 95, 82, 83];
% goals = [{[54, 36]}; {[17, 87]}; {[64, 28]}];% agent per row
% goals1Dem = [54, 36, 17, 87, 64, 28];

% %agents and obstacles information 2
% start = [5; 41; 49]; % per agent
% obstacles = [16 62 65 33 37 91 100];
% goals = [{75}; {48}; {42}];% agent per row
% goals1Dem = [75, 48, 42];

% %agents and obstacles information 3
% start = [31;41;3]; % per agent
% obstacles = [2, 13, 56, 58, 86, 88, 77, 24, 35, 45, 94, 80, 70, 69, 48, 38, 95, 82, 83, 59];
% goals = [{[54, 36]}; {[64, 27]}; {[17, 87]}];% agent per row
% goals1Dem = [54, 36, 64, 27, 17, 87];

% %agents and obstacles information 4
% start = [12;91;10]; % agent per row
% obstacles = [13, 56, 86, 88, 77, 24, 35, 45, 83, 84, 80, 70, 69, 48, 38, 95];
% goals = [{[25, 54]}; {[94, 100]}; {[17, 87]}];% agent per row
% goals1Dem = [25, 54, 94, 100, 17, 87];

% agents and obstacles information 5
start = [12;91;20]; % agent per row
obstacles = [13, 56, 86, 88, 77, 24, 35, 45, 83, 84, 80, 70, 69, 48, 38, 95];
goals = [{[25, 54]}; {[94, 100]}; {[17, 87]}];% agent per row
goals1Dem = [25, 54, 94, 100, 17, 87];

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
filePath = 'D:\livelockAutonomous2\SymCodes';

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
% plot(xStart(1),yStart(1),'ob','MarkerSize',12,'LineWidth',2) % plot agents as black circles
% plot(xStart(2),yStart(2),'og','MarkerSize',12,'LineWidth',2)
% plot(xStart(3),yStart(3),'or','MarkerSize',12,'LineWidth',2)
%plot start point
goalPlot(xGoal,yGoal,.4,'m');
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
colorBox = ['b' 'g' 'r'];

for m = 1:M % for each agent, m
    expansion = -1; % used to expand gridChunk if required
    while isempty(cPath{m}) % while no feasible plan exists for current gridChunk
        expansion = expansion + 1; % expand the grid
        gridChunk(m,:) = getChunk(start(m), goals{m}, gridWidth, gridLength, expansion);
        hold on % still on Figure 1, planning environment
%         rectangle('Position',[gridChunk(m,1)-.5, gridChunk(m,3)-.5, ...
%             gridChunk(m,2)-gridChunk(m,1)+1, gridChunk(m,4)-gridChunk(m,3)+1],...
%             'EdgeColor',colorBox(m),'LineWidth',3)
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
IndexNum = zeros(M,1);
for m = 1:M
    cEnd = find(ismember(cPath{m},goalsLeft{m}),1,'last');
    plot(xPath{m}(1),yPath{m}(1),[colorBox(m) 'o'],'MarkerSize',12) % Plot start and end points of path
    arrow([xPath{m}(1:cEnd-1)' - (m-2)*0.1,yPath{m}(1:cEnd-1)' - (m-2)*0.1],...
        [xPath{m}(2:cEnd)' - (m-2)*0.1,yPath{m}(2:cEnd)'- (m-2)*0.1],8,'Color',colorBox(m),'LineWidth',1); % plot path as arrows
    rectangle('Position',[gridChunk(m,1)-.5, gridChunk(m,3)-.5, ...
        gridChunk(m,2)-gridChunk(m,1)+1, gridChunk(m,4)-gridChunk(m,3)+1],...
        'EdgeColor',colorBox(m),'LineWidth',3); % plot final gridChunk
    IndexNum(m,1) = text(x{m}(1,1)+.15,x{m}(2,1)+.25,num2str(m));
    set(IndexNum(m,1),'fontsize',16)
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
for cycle = 1:length(obstacles)
    rectangle('Position',[mod(obstacles(cycle) - 1,10) + .5, floor((obstacles(cycle) - 1)/10) + .5, 1, 1],'FaceColor',[1 .99 .9]);
end   
pos = zeros(M,3); % stores agent plot marker.  used for delete function to update marker
for m = 1:M
    %     pos(m,1) = plot(x{m}(1,1),x{m}(2,1),'ko','MarkerSize',12);
    pos(m,1) = circle(x{m}(1,1),x{m}(2,1),mR,[colorBox(m) '-']);
    pos(m,2) = text(x{m}(1,1)+.15,x{m}(2,1)+.25,num2str(m));
    set(pos(m,2),'fontsize',16)
    pos(m,3) = circle(x{m}(1,1),x{m}(2,1),sR,[colorBox(m) '--']);
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
        %         obsDisAll(m,:) = sqrt((xObs - x{m}(1,k)).^2 + (yObs - x{m}(2,k)).^2); % obstacle distance
        %         obsComplex(m,:) = sum(obsDisAll(m,:)<1.1); % within inside the sensing range
        
        % check for obstacles around you %YW: based on discrete state,
        % equivalent to observation size can sense 8 neighboring cells,
        % commmunication also 8 neighboring cells
        cSurround = [cPath{m}(pointCounter(m)-1)-1, cPath{m}(pointCounter(m)-1)+1, cPath{m}(pointCounter(m)-1)-gridWidth, cPath{m}(pointCounter(m)-1)+gridWidth];
        % sensedObstacles{m} = [sensedObstacles{m}, cSurround(ismember(cSurround,obstacles))];
        % if any adjacent cells contain an obstacle
        if sum(ismember(cSurround,obstacles(~ismember(obstacles,sensedObstacles{m})))) > 0
            sensedObstacles{m} = [sensedObstacles{m}, cSurround(ismember(cSurround,obstacles(~ismember(obstacles,sensedObstacles{m}))))];
            [xSObs{m},ySObs{m}] = cellPath2Grid(sensedObstacles{m},gridWidth, gridLength);
            delete(hObs{m});
            hObs{m} = obsPlot(xSObs{m},ySObs{m},1.0,[0.5 0.5 0],2);
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
    
    %% 2. Robot Collision detection method 1
    
    for m = 1:M % for each agent
        if pointCounter(m)>numel(cPath{m})%%YW: in NuSMV: from start to final goal, it will return 2 more steps. we make sure that they are equal to the final state
            cPath{m}(pointCounter(m)) = cPath{m}(pointCounter(m)-2);
        end
        mTrack{m} = cPath{m}(pointCounter(m)-1:pointCounter(m)); % update collision tracker %YW: record pos for the robot m, current pos cPath{m}(pointCounter(m)-1} and next pos cPath{m}(pointCounter(m))
    end
    
    for m = 1:M
        for m2 = 1:M
            if m2 ~= m
                if ismember(cPath{m}(pointCounter(m)),mTrack{m2}(2))% collision into same place %%YW: because mTrack{m2} = cPath{m2}(pointCounter(m2)-1:pointCounter(m2)), mTrack{m2}(2) is the next pos of m2, hence collision type -1 represents both robots go to the same cell at the next step
                    mCollision{m}(m2) = -1; % YW: one robot wait for another one to pass, and it is always the robot with smaller index m that waits
                    
                    if ismember(mTrack{m}(2),cPath{m2}(pointCounter(m2)-1)) && cPath{m2}(pointCounter(m2)-1) == cPath{m2}(pointCounter(m2))
                        mCollision{m}(m2) = -2; %YW: run into each other, or one of them wait
                    elseif ismember(mTrack{m2}(2),cPath{m}(pointCounter(m)-1)) && cPath{m}(pointCounter(m)-1) == cPath{m}(pointCounter(m))
                        mCollision{m}(m2) = -2; % used to release to no collision codition 0 and avoid replan twice, see e.g. Line 336,348
                    end
                    
                elseif ismember(mTrack{m2}(2),cPath{m}(pointCounter(m)-1)) && ismember(mTrack{m}(2),cPath{m2}(pointCounter(m2)-1))
                    mCollision{m}(m2) = -2;
                else
                    mCollision{m}(m2) = 0;
                end
            end
        end
    end
    
    % complicated collision1
    for m = 1:M
        if sum(mCollision{m}) == -3
            m2 = find(mCollision{m} == -2); %YW: not the m2 above
            m1 = find(mCollision{m} == -1);
            if ~isempty(m2)&&~isempty(m1)
                Goalsexchange = goalsLeft{m};
                goalsLeft{m} = goalsLeft{m2};
                goalsLeft{m2} = goalsLeft{m1};
                goalsLeft{m1} = Goalsexchange;
                
                goals{m} = [goalsReached{m}, goalsLeft{m}];
                goals{m2} = [goalsReached{m2}, goalsLeft{m2}];
                goals{m1} = [goalsReached{m1}, goalsLeft{m1}];
                
                CollisionReplanScript_v3; % Replan (seperated for sanity)
                pause(1)
                CollisionReplanScript_v3_m2;
                pause(1)
                CollisionReplanScript_v3_m1;
                pause(1)
                mCollision{m}(m2) = 0;
                mCollision{m}(m1) = 0;
                mCollision{m2}(m) = 0;
                mCollision{m2}(m1) = 0;
                mCollision{m1}(m) = 0;
                mCollision{m1}(m2) = 0;
            end
        end
    end
    
    % complicated collision 2
    if cPath{1}(pointCounter(1)) == cPath{2}(pointCounter(2)-1) && mCollision{2}(3) == -1
        Goalsexchange = goalsLeft{1};
        goalsLeft{1} = goalsLeft{3};
        goalsLeft{3} = goalsLeft{2};
        goalsLeft{2} = Goalsexchange;
        
        m = 1;m2 = 2;m1 = 3;
        goals{m} = [goalsReached{m}, goalsLeft{m}];
        goals{m2} = [goalsReached{m2}, goalsLeft{m2}];
        goals{m1} = [goalsReached{m1}, goalsLeft{m1}];
        
        CollisionReplanScript_v3; % Replan (seperated for sanity)
        pause(1)
        CollisionReplanScript_v3_m2;
        pause(1)
        CollisionReplanScript_v3_m1;
        pause(1)
        mCollision{m}(m2) = 0;
        mCollision{m}(m1) = 0;
        mCollision{m2}(m) = 0;
        mCollision{m2}(m1) = 0;
        mCollision{m1}(m) = 0;
        mCollision{m1}(m2) = 0;
    end
    
    % ordinary collision
    for m = 1:M
        for m2 = 1:M
            if m2 ~= m
                if mCollision{m}(m2) == -1 % YW: conllision avoidance situation -1, wait, insert a state in the planned path
                    for renewNum = length(cPath{m})+1:-1:pointCounter(m)
                        cPath{m}(renewNum) = cPath{m}(renewNum-1);
                    end
                    [xPath{m}, yPath{m}]= cellPath2Grid(cPath{m},gridWidth, gridLength);
                    ReplottingScript
                    pause(1)
                    mCollision{m2}(m) = 0;
                elseif mCollision{m}(m2) == -2
                    % exchange goals
%                     Goalsexchange = goalsLeft{m};
%                     goalsLeft{m} = goalsLeft{m2};
%                     goalsLeft{m2} = Goalsexchange; %YW: sense another
%                     robot as an obstacle, simply replan
                    sensedObstacles{m} = [sensedObstacles{m}, cPath{m2}(pointCounter(m2)-1)];
                    sensedObstacles{m2} = [sensedObstacles{m2}, cPath{m}(pointCounter(m)-1)];
                    goals{m} = [goalsReached{m}, goalsLeft{m}];
                    goals{m2} = [goalsReached{m2}, goalsLeft{m2}];
                    pause(1)
                    CollisionReplanScript_v3; % Replan (seperated for sanity)
                    pause(1)
                    CollisionReplanScript_v3_m2;
                    pause(1)
                    mCollision{m2}(m) = 0;
                    sensedObstacles{m}(sensedObstacles{m} == cPath{m2}(pointCounter(m2)-1)) = [];
                    sensedObstacles{m2}(sensedObstacles{m2} == cPath{m}(pointCounter(m)-1)) = [];
                end
            end
        end
    end
    
%     %% method 2
%     for m = 1:M % for each agent
%         if pointCounter(m)>numel(cPath{m})
%             cPath{m}(pointCounter(m)) = cPath{m}(pointCounter(m)-2);
%         end
%         mTrack{m} = cPath{m}(pointCounter(m)-1:pointCounter(m)); % update collision tracker
%     end
%     
%     for m = 1:M
%         for m2 = 1:M
%             if m2 ~= m
%                 if ismember(cPath{m}(pointCounter(m)),mTrack{m2}(2))% collision into same place
%                     mCollision{m}(m2) = -1;
%                     
%                     if ismember(mTrack{m}(2),cPath{m2}(pointCounter(m2)-1)) && cPath{m2}(pointCounter(m2)-1) == cPath{m2}(pointCounter(m2))
%                         mCollision{m}(m2) = -2;
%                     elseif ismember(mTrack{m2}(2),cPath{m}(pointCounter(m)-1)) && cPath{m}(pointCounter(m)-1) == cPath{m}(pointCounter(m))
%                         mCollision{m}(m2) = -2;
%                     end
%                     
%                     if mCollision{m}(m2) == -1
%                         for renewNum = length(cPath{m})+1:-1:pointCounter(m)
%                             cPath{m}(renewNum) = cPath{m}(renewNum-1);
%                         end
%                         [xPath{m}, yPath{m}]= cellPath2Grid(cPath{m},gridWidth, gridLength);
%                         mCollision{m2}(m) = 0;
%                     elseif mCollision{m}(m2) == -2
%                         % exchange goals
%                         Goalsexchange = goalsLeft{m};
%                         goalsLeft{m} = goalsLeft{m2};
%                         goalsLeft{m2} = Goalsexchange;
%                         goals{m} = [goalsReached{m}, goalsLeft{m}];
%                         goals{m2} = [goalsReached{m2}, goalsLeft{m2}];
%                         CollisionReplanScript_v3; % Replan (seperated for sanity)
%                         CollisionReplanScript_v3_m2;
%                         mCollision{m2}(m) = 0;
%                     end
%                     
%                 elseif ismember(mTrack{m2}(2),cPath{m}(pointCounter(m)-1)) && ismember(mTrack{m}(2),cPath{m2}(pointCounter(m2)-1))
%                     mCollision{m}(m2) = -2;
%                     Goalsexchange = goalsLeft{m};
%                     goalsLeft{m} = goalsLeft{m2};
%                     goalsLeft{m2} = Goalsexchange;
%                     goals{m} = [goalsReached{m}, goalsLeft{m}];
%                     goals{m2} = [goalsReached{m2}, goalsLeft{m2}];
%                     CollisionReplanScript_v3; % Replan (seperated for sanity)
%                     CollisionReplanScript_v3_m2;
%                     mCollision{m2}(m) = 0;
%                 else
%                     mCollision{m}(m2) = 0;
%                 end
%             end
%         end
%     end
    
    %% 3. LQR
    stepFish = zeros(M,1); %YW: finish the move from current cell to next cell
    
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
            pos(m,1) = circle(x{m}(1,k+1),x{m}(2,k+1),mR,[colorBox(m) '-']);
            pos(m,2) = text(x{m}(1,k+1)+.15,x{m}(2,k+1)+.25,num2str(m)); % update agent label
            set(pos(m,2),'fontsize',16)
            pos(m,3) = circle(x{m}(1,k+1),x{m}(2,k+1),sR,[colorBox(m) '--']);
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
                
                %goalsLeft{m} = goals{m}(ismember(goals{m},goalsReached{m})==0);
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
% plot(xGoal,yGoal,'dm','MarkerSize',12)
goalPlot(xGoal,yGoal,.4,'m');
% plot(xObs,yObs,'xr','MarkerSize',15)
%  obsPlot(xObs,yObs,.3,'k',1);
for m = 1:M
    plot(xStart(m),yStart(m),[colorBox(m) 's'],'LineWidth',2)
    plot(x{m}(1,1:k),x{m}(2,1:k),[colorBox(m) '-'],'LineWidth',1.5)
end
hold off
