function []=animate(l,t1,t2,t3)

%Calculation of joints
x1=zeros(1,100);
y1=zeros(1,100);
x2=l(1)*cos(t1);
y2=l(1)*sin(t1);
x3=l(1)*cos(t1) + l(2)*cos(t1+t2);
y3=l(1)*sin(t1) + l(2)*sin(t1+t2);
x4=l(1)*cos(t1) + l(2)*cos(t1+t2) + l(3)*cos(t1+t2+t3);
y4=l(1)*sin(t1) + l(3)*sin(t1+t2) + l(3)*sin(t1+t2+t3);

%Plotting of the joint coordinates
for i=1:100
    
    plot([x1(i),x2(i)],[y1(i),y2(i)],'-ro')
    grid on
    title('Simulation of three link manipulator')
     hold on
     axis([-3 3 -3 3])
     axis square
    plot([x2(i),x3(i)],[y2(i),y3(i)],'-bo')
    plot([x3(i),x4(i)],[y3(i),y4(i)],'-m')
    pause(0.01)
    hold off
    
end