% CollisionReplanScript

%% Replanning

newStart(m2) = cPath{m2}(pointCounter(m2)-1);
% sensedObstacles{m2} = [sensedObstacles{m2}, cPath{m2}(pointCounter(m2))];

% rewrite specification
ltlspec{m2} = ['LTLSPEC ! (( F (x.state = ',num2str(goalsLeft{m2}(1)),')'];%,' & F (x.state = 87) & F(x.state = 54)) )'];
for i = 2:length(goalsLeft{m2})
    ltlspec{m2} = [ltlspec{m2}, ' & F (x.state = ',num2str(goalsLeft{m2}(i)),')'];
end
ltlspec{m2} = [ltlspec{m2}, ' ))'];

% recalculate path
cPath{m2} = [];
expansion = -1;
while isempty(cPath{m2})
    expansion = expansion + 1;
    gridChunk(m2,:) = getChunk(newStart(m2), goalsLeft{m2}, gridWidth, gridLength, expansion);
    makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m2,:), newStart(m2), sensedObstacles{m2}, ltlspec{m2});
    % Replan according to new sensor information
    [~, output] = system(['NuSMV ' filePath '\' fileName]);
    [xPath{m2},yPath{m2},cPath{m2}] = getPath(output, gridWidth, gridLength);
end

pointCounter(m2) = 2; % reset pointCounter

% update sensed obstacle positions
[xSObs{m2},ySObs{m2}] = cellPath2Grid(sensedObstacles{m2},gridWidth, gridLength);

%% Replotting 
ReplottingScript

%% Return to main loop