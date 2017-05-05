function[] = makeSMV_v2(fileName, gridWidth, gridLength, gridChunk, start, obstacles, ltlspec)
%function[] = makeSMV(fileName, gridWidth, gridLength, start, obstacles)
%   fileName = string title of SMV, i.e. 'fileName.smv'
%   gridWidth = number of cells along x
%   gridLength = number of cells along y
%   start = initial cell
%   obstacles = vector of forbidden cells

fid = fopen(fileName,'w'); % open file

fprintf(fid, 'MODULE main\n');
fprintf(fid, 'VAR\n');
fprintf(fid, 'x : grid;\n');
fprintf(fid, '%s\n',ltlspec); % Write specification from input
fprintf(fid, 'MODULE grid\n');
fprintf(fid, 'VAR\n');
% states = [3641:3650, 3731:3740, 3821:3830, 3911:3920, 4001:4010, 4091:4100, 4181:4190, 4271:4280, 4361:4370, 4451:4460];
states = genStates(gridChunk, gridWidth, gridLength);
states =  regexprep(num2str(states),'\s*',', ');
fprintf(fid,'state : {%s} ;\n',states);
fprintf(fid, 'ASSIGN\n');
fprintf(fid, 'init(state) := %.0f;\n',start); % starting position
fprintf(fid, 'next(state) :=\n');
fprintf(fid, 'case\n');
% Run function to generate state transition rules
printGridTransitions_v2(fid,gridWidth, gridLength, gridChunk, obstacles);
fprintf(fid, 'TRUE : state;\n');
fprintf(fid, 'esac;');

fclose(fid); % close file

















