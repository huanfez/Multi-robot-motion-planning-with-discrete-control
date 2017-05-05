function[transitionArray] = printGridTransitions_v2(fid, gridWidth, gridLength, gridChunk, obstacles)
% function = printGridTransitions(fid, gridWidth, gridLength)
%   fid = file id to write to (from fopen)
%   gridWidth = Width (x) of cell grid
%   gridLength =  Length(y) of cell grid
%   gridChunck = cell partition being considered in model in abs. ref.
%       gridChunk = [xl xu yl yu] (NOT Relative)
%       Note: leftmost = 1, rightmost = gridWidth/Length
%   obstacles = location of obstacles as cell references
%   
%   This function writes the state transition rules for the NuSMV model
%   

xl = gridChunk(1);
xu = gridChunk(2);
yl = gridChunk(3);
yu = gridChunk(4);

transitionArray = cell(gridWidth*gridLength,1);

for i = 1:gridWidth*gridLength
    x = mod((i-1),gridWidth)+1;
    y = floor((i-1)/gridWidth)+1;
    
    % I am leaving in state = obstacle just in case something happens that
    % puts the vehicle in an obstacle state.  To remove add:
    % if ismember(i,obstacles). (i.e. if i is in obstacles) and
    % replace 'if' below with 'elseif'
    
    if x < xl || x > xu || y < yl || y > yu
        % If outside of gridChunk, ignore it
        transition = [];
    elseif x > xl && x < xu
        if y > yl && y < yu
            transition = [i-gridWidth, i-1, i+1, i+gridWidth];
        elseif y == yl
            transition = [i-1, i+1, i+gridWidth];
        else % y == yu
            transition = [i-gridWidth, i-1, i+1];
        end
    elseif x == xl
        if y > yl && y < yu
            transition = [i-gridWidth, i+1, i+gridWidth];
        elseif y == yl
            transition = [i+1, i+gridWidth];
        else % y == yu
            transition = [i-gridWidth, i+1];
        end
    else % x == xu
        if y > yl && y < yu
            transition = [i-gridWidth, i-1, i+gridWidth];
        elseif y == yl
            transition = [i-1, i+gridWidth];
        else % y == yu
            transition = [i-gridWidth, i-1];
        end
    end
    
%     transition = transition(ismember(transition,obstacles)==0);

    % First, remove obstacles.  Then add commas
    transitionArray(i) = {regexprep(num2str(transition(ismember(transition,obstacles)==0)),'\s*',', ')};
    % Write states to the smv file
    if ~isempty(transitionArray{i}) %%%
    fprintf(fid, 'state = %.0f : {%s};\n',i,transitionArray{i});
    end%%%
    
end
