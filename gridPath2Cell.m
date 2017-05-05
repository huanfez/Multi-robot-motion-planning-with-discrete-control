function [c] = gridPath2Cell(xPath, yPath, gridWidth, gridLength)
% function [c] = gridPath2Cell(xPath, yPath, gridWidth, gridLength)


c = gridWidth.*(yPath-1) + xPath;

if ~isempty(c(c>gridLength*gridWidth))
    fprintf('\nCaution: path exceeds grid dimensions!\n')
    fprintf('Press Enter to continue\n')
    pause
end

