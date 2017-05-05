% CollisionReplanScript

%% Replanning
ltlspec = [];
ltlspecS = cell(3);
for m = 1:M
    for m2 = 1:M
        if m2 > m
            if mCollision{m}(m2) == -1
                % update SMV start position
                newStart(m) = cPath{m}(pointCounter(m)-1); 
                newStart(m2) = cPath{m2}(pointCounter(m2)-1);
                % Rewrite specification
                ltlspecS{m,m2} = ['F (x',num2str(m),'.state = ',num2str(tempFinalState(m)),')','&F (x', ...
                    num2str(m2),'.state = ',num2str(tempFinalState(m2)),')','&G !(x',num2str(m),'.state = x',num2str(m2),'.state)'];
                if ~isempty(ltlspecS{m,m2})
                    ltlspec = [ltlspec,'&',ltlspecS{m,m2}];
                end
            end
        end
    end
end
ltlspec(1) = [];
ltlspec = ['LTLSPEC !(',ltlspec, ')'];

minLocY = min(LocRegion/10);
minLocX = min(mod(LocRegion,10));
maxLocY = max(LocRegion/10);
maxLocX = max(mod(LocRegion,10));

LocStart = minLocX + minLocY*10;
LocGoal = maxLocX + maxLocY*10;

% Recalculate path
cPathNewPie = []; % empty cPath
expansion = -1; % reset expansion
while isempty(cPathNewPie) % while no path is found
    expansion = expansion + 1;
    gridChunkS = getChunk(LocStart, LocGoal, gridWidth, gridLength, expansion);
    makeSMV_v3(fileNameS, gridWidth, gridLength, gridChunkS, newStart, sensedObstacles, ltlspec);
    % Add conflicting agent(s) to obstacles list for this instance only
    [~,output] = system(['cd ' pathNuSMV ' & NuSMV ' filePath '\' fileNameS]);
    % pick out path of the one that have collision
    [agent1,agent2] = find(mCollision);
    for m = 1:M
        if ismember(m,[agent1, agent2])
            [xPathNewPie{m},yPathNewPie{m},cPathNewPie{m}] = getPathCollision(output, gridWidth, gridLength, m);
            cPath{m}((pointCounter(m)-1):(pointCounter(m)- 1 + tempNum(m))) = [];
            cPath{m}((pointCounter(m)-1):end) = [cPathNewPie{m}, cPath{m}((pointCounter(m)-1):end)];
        end
    end
end
[xPath, yPath] = cellPath2Grid(cPath,gridWidth, gridLength);
pointCounter(m) = 2; % reset pointCounter

%% Replotting
ReplottingScript

%% Return to main loop