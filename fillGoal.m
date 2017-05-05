function h = fillGoal(cell,w,gridWidth,gridLength,color)
% cell = cell number
% color = e.g. 'm'

[x,y] = cellPath2Grid(cell,gridWidth,gridLength);
l = 5*w/4;

x1 = [    x(1); x(1)+w/2;     x(1); x(1)-w/2];
y1 = [y(1)+l/2;     y(1); y(1)-l/2;     y(1)];

h = fill(x1,y1,color);

if length(cell) > 2
    for i = 2:length(cell)
        x1 = [    x(i); x(i)+w/2;     x(i); x(i)-w/2];
        y1 = [y(i)+l/2;     y(i); y(i)-l/2;     y(i)];
        fill(x1,y1,color)
    end
end