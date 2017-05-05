% ObstacleReplanScript

%% Replanning

newStart(m) = cPath{m}(pointCounter(m)-1);
% sensedObstacles{m} = [sensedObstacles{m}, cPath{m}(pointCounter(m))];

% rewrite specification
ltlspec{m} = ['LTLSPEC ! (( F (x.state = ',num2str(goalsLeft{m}(1)),')'];%,' & F (x.state = 87) & F(x.state = 54)) )'];
for i = 2:length(goalsLeft{m})
    ltlspec{m} = [ltlspec{m}, ' & F (x.state = ',num2str(goalsLeft{m}(i)),')'];
end
ltlspec{m} = [ltlspec{m}, ' ))'];

% recalculate path
cPath{m} = [];
expansion = -1;
while isempty(cPath{m})
    expansion = expansion + 1;
    gridChunk(m,:) = getChunk(newStart(m), goalsLeft{m}, gridWidth, gridLength, expansion);
    makeSMV_v2(fileName, gridWidth, gridLength, gridChunk(m,:), newStart(m), sensedObstacles{m}, ltlspec{m});
    % Replan according to new sensor information
    [~,output] = system(['NuSMV ' filePath '\' fileName]);
    [xPath{m},yPath{m},cPath{m}] = getPath(output, gridWidth, gridLength);
end

pointCounter(m) = 2; % reset pointCounter

% update sensed obstacle positions
[xSObs{m},ySObs{m}] = cellPath2Grid(sensedObstacles{m},gridWidth, gridLength);

%% Replotting 
ReplottingScript
%% Return to main loop