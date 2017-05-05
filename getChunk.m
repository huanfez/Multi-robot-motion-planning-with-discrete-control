function[gridChunk] = getChunk(start, goals, gridWidth, gridLength, expansion)

[x,y] = cellPath2Grid([start goals], gridWidth, gridLength);

gridChunk = [min(x)-expansion max(x)+expansion min(y)-expansion max(y)+expansion];

if gridChunk(1) < 1
    gridChunk(1) = 1;
end
if gridChunk(2) > gridWidth
    gridChunk(2) = gridWidth;
end
if gridChunk(3) < 1
    gridChunk(3) = 1;
end
if gridChunk(4) > gridLength
    gridChunk(4) = gridLength;
end