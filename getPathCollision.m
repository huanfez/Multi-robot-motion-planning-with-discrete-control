function[xPath, yPath, cPath] = getPathCollision(output, gridWidth, gridLength, num)
% function[xPath, yPath, cPath] = getPath(output, gridWidth, gridLength)
%   Takes NuSMV output and uses regexp to pull out path of cells.  Then
%   uses discretization (gridWidth, gridLength) to convert those cells to
%   x,y coordinates.
%       xPath - list of x-coordinates
%       yPath - list of y-coordinates
%       cPath - path listed as cell numbers
if num == 1
    t = regexp(output, '  x1.state = (\d*)', 'tokens');
elseif num == 2
    t = regexp(output, '  x2.state = (\d*)', 'tokens');
elseif num == 3
    t = regexp(output, '  x3.state = (\d*)', 'tokens');
end

cPath = zeros(1,length(t));

for i = 1:length(t)
    cPath(i) = str2double(t{i}{1});
    % regexp seems to output it as a cell of cells.  There is likely a
    % better way to do this...
end

[xPath, yPath] = cellPath2Grid(cPath,gridWidth, gridLength);
% command that converts cell path to x,y path