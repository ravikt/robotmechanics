%Inverse kinematic simulation of three link robot
%Trajectory generation is done using quintic polynomial
%DH parameters using Modified DH approach
%Sum of all the joint angles has been taken pi/4
clear all;
close all;
syms t
x = [1  t  t^2  t^3  t^4  t^5];
r1 = subs(x,0);
r2 = subs(diff(x),0);
r3 = subs(diff(diff(x)),0);
r4 = subs(x,10);
r5 = subs(diff(x),10);
r6 = subs(diff(diff(x)),10);
A  = [r1;r2;r3;r4;r5;r6];
B  = [0.5;0;0;1;1;0];
c  = pinv(A)*B;

%Trajectory equation
tx = c(1) + c(2)*t + c(3)*t^2 +c(4)*t^3 + c(5)*t^4 + c(6)*t^5;
ty = c(1) + c(2)*t + c(3)*t^2 +c(4)*t^3 + c(5)*t^4 + c(6)*t^5;

t = linspace(0,10);
xe = subs(tx,t);%linspace(0.5,1.2);
ye = subs(ty,t);%linspace(0.5,0.9);
vx= subs(diff(tx),t);
vy= subs(diff(ty),t);
n = size(xe,2);
theta1 = zeros(1,n);
theta2 = zeros(1,n);
theta3 = zeros(1,n);
c2     = zeros(1,n);
s2     = zeros(1,n);
sphi   = 0.7071;
cphi   = 0.7071;

%Calculation of all the three joint variables
%Closed form solution using algebraic approach

for i=1:n

%Calculation of theta2
c2(i)    = ((xe(i)^2+ye(i)^2) - 2)/2;
s2(i)    = sqrt(1-c2(i)^2);
theta2(i)= atan2(s2(i),c2(i));
%Calculation of theta1
k1(i) = 1+1*c2(i);
k2(i) = 1*s2(i);
r(i)  = sqrt(k1(i)^2+k2(i)^2);
gamma(i) = atan2(k2(i),k1(i));
theta1(i) = atan2(xe(i),ye(i))-atan2(k2(i),k1(i));
%Calculation of theta3
theta3(i) = atan2(sphi,cphi)-theta1(i)-theta2(i);
end

%Calculation of joints
x1=zeros(1,n);
y1=zeros(1,n);
x2=cos(theta1);
y2=sin(theta1);

x3=cos(theta1)+cos(theta1+theta2);
y3=sin(theta1)+sin(theta1+theta2);
x4=x3+0.707;
y4=y3+0.707;

%Plotting of the joint coordinates

for i=1:n
    
    plot([x1(i),x2(i)],[y1(i),y2(i)],'-ro')
    grid on
    hold on
    axis([-2 2 -2 2])
    axis square
    plot([x2(i),x3(i)],[y2(i),y3(i)],'-bo')
                   
    plot([x3(i),x4(i)],[y3(i),y4(i)],'-m')
                   
    pause(0.1)
    hold off
   
    
end



%End effector trajectory

plot3(t,x4,y4)
title('End-effector trajectory')
grid on