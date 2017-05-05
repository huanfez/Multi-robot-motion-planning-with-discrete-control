function h = obsPlot(x,y,w,color,LineWidth)
% x,y = array of positions
% w = width of obs (x) square
% color = e.g. 'r'

if size(x,2) == 1
    x = x';
    y = y';
end
% 
% x1 = [[(x-w/2)';(x-w/2)'] [(x+w/2)';(x+w/2)']];
% y1 = [[(y+w/2)';(y-w/2)'] [(y-w/2)';(y+w/2)']];

x1 = [(x-w/2) (x-w/2); (x+w/2) (x+w/2)];
y1 = [(y+w/2) (y-w/2); (y-w/2) (y+w/2)];

h = plot(x1,y1,color,'LineWidth',LineWidth);

