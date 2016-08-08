% The Desperate Space robot
% May 1st, 2015

x0= rand(1,100);%[-1 -2 -3 -4 -5 -6];
y0= rand(1,100);%[0 0 0 0 0 0];

for i=1:100
x=[x0(i)-1,x0(i)+1,x0(i)+1,x0(i)-1,x0(i)-1];
y=[y0(i)+1,y0(i)+1,y0(i)-1,y0(i)-1,y0(i)+1];

l=[1 1 1];
%Left arm
t=[pi/2+pi/4 -pi/2 -pi/2];% Fixed orientation of the arm
lx1=x0(i)+0;
ly1=y0(i)+1;              % Fix the left arm to the base
lx2=lx1+l(1)*cos(t(1));
ly2=ly1+l(1)*sin(t(1));
lx3=lx1+l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2));
ly3=ly1+l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2));
lx4=lx1+l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2)) + l(3)*cos(t(1)+t(2)+t(3));
ly4=ly1+l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2)) + l(3)*sin(t(1)+t(2)+t(3));

%Right arm
t=[-pi/2-pi/4 pi/2 pi/2]; % Fixed orientation of the arm
rx1=x0(i)+0;
ry1=y0(i)-1;              % Fix the right arm to the base
rx2=rx1+l(1)*cos(t(1));
ry2=ry1+l(1)*sin(t(1));
rx3=rx1+l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2));
ry3=ry1+l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2));
rx4=rx1+l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2)) + l(3)*cos(t(1)+t(2)+t(3));
ry4=ry1+l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2)) + l(3)*sin(t(1)+t(2)+t(3));

plot(x,y)
hold on
plot(x0(i),y0(i),'-ro')
axis([-10 10 -10 10])

hold on
plot([lx1,lx2,lx3,lx4],[ly1,ly2,ly3,ly4],'-r')
hold on
plot([rx1,rx2,rx3,rx4],[ry1,ry2,ry3,ry4],'-b')
hold on
pause(0.2)
hold off
end