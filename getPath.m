function[xPath, yPath, cPath] = getPath(output, gridWidth, gridLength)
% function[xPath, yPath, cPath] = getPath(output, gridWidth, gridLength)
%   Takes NuSMV output and uses regexp to pull out path of cells.  Then
%   uses discretization (gridWidth, gridLength) to convert those cells to
%   x,y coordinates.
%       xPath - list of x-coordinates
%       yPath - list of y-coordinates
%       cPath - path listed as cell numbers

t = regexp(output, '  x.state = (\d*)', 'tokens');

cPath = zeros(1,length(t));

for i = 1:length(t)
    cPath(i) = str2double(t{i}{1});
    % regexp seems to output it as a cell of cells.  There is likely a
    % better way to do this...
end

[xPath, yPath] = cellPath2Grid(cPath,gridWidth, gridLength);
% command that converts cell path to x,y path