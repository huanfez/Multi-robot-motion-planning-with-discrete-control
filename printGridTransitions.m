function[transitionArray] = printGridTransitions(fid, gridWidth, gridLength, obstacles)
% function = printGridTransitions(fid, gridWidth, gridLength)
%   fid = file id to write to (from fopen)
%   gridWidth = Width (x) of cell grid
%   gridLength =  Length(y) of cell grid
%   
%   Write the state transition rules for the NuSMV model
%   

transitionArray = cell(gridWidth*gridLength,1);

for i = 1:gridWidth*gridLength
    x = mod((i-1),gridWidth)+1;
    y = floor((i-1)/gridWidth)+1;
    
    % I am leaving in state = obstacle just in case something happens that
    % puts the vehicle in an obstacle state.  To remove add:
    % if ismember(i,obstacles). (i.e. if i is in obstacles) and
    % replace 'if' below with 'elseif'
    
    if x > 1 && x < gridWidth
        if y > 1 && y < gridLength
            transition = [i-gridWidth, i-1, i+1, i+gridWidth];
        elseif y == 1
            transition = [i-1, i+1, i+gridWidth];
        else % y == gridLength
            transition = [i-gridWidth, i-1, i+1];
        end
    elseif x == 1
        if y > 1 && y < gridLength
            transition = [i-gridWidth, i+1, i+gridWidth];
        elseif y == 1
            transition = [i+1, i+gridWidth];
        else % y == gridLength
            transition = [i-gridWidth, i+1];
        end
    else % x == gridWidth
        if y > 1 && y < gridLength
            transition = [i-gridWidth, i-1, i+gridWidth];
        elseif y == 1
            transition = [i-1, i+gridWidth];
        else % y == gridLength
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




































