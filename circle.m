function h = circle(x,y,r,style)
% based on rectangle function
% h = handle
% x,y = position
% r = radius
% style = plot style string (e.g. 'k--')

% d = 2*r;
% px = x-r;
% py = y-r;
% h = rectangle('Curvature',[1 1],'Position',[px py d d]);
% daspect([1 1 1])

th = 0:pi/50:2*pi;
xunit = r*cos(th) + x;
yunit = r*sin(th) + y;
h = plot(xunit,yunit,style);
