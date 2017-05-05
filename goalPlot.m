function h = goalPlot(x,y,w,color)
% x,y = array of positions
% w = width of obs (x) square
% color = e.g. 'r'

l = 5*w/4;

if size(x,2) == 1
    x = x';
    y = y';
end

x1 = [    x; x+w/2;     x; x-w/2;     x; x+w/2];
y1 = [y+l/2;     y; y-l/2;     y; y+l/2;     y];

h = plot(x1,y1,color);%,'LineWidth',2);