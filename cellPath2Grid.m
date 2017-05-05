function [x,y] = cellPath2Grid(cellPath, gridWidth, gridLength)
% function [x,y] = cellPath2Grid(path, gridWidth, gridLength
%    cellPath = cell numbers (integers) from path planning
%    gridWidth = number of x (horizontal) cells
%    gridLength = number of y (verticle) cells
%    cell style for gL=3 x gW=4
%    |  9 10 11 12 |
%    |  5  6  7  8 |
%    |  1  2  3  4 |

if ~isempty(cellPath(cellPath>gridLength*gridWidth))
    fprintf('\nCaution: path exceeds grid dimensions!\n')
    fprintf('Press Enter to continue\n')
    pause
end

x = mod((cellPath-1),gridWidth)+1;
y = floor((cellPath-1)/gridWidth)+1;

























