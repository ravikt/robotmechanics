[t,x] = ode45(@sample,[0,3],[0,0]);
theta = x(:,1);
r     = 1;
x2    = r*cos(theta);
y2    = r*sin(theta);
x1    = 0;
y1    = 0;

for i=1:137
    plot([x1,x2(i)],[y1,y2(i)])
    axis([-2 2 -2 2])
    hold off;
    pause(0.1);
end