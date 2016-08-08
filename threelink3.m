%Simulation of three link planar manipulator
%Involves calculation of Jacobian for each configuration
%The calculation is done using ode45
%A function jvelo is defined to perform the integration.
%Initial configuration for phi = pi/4 is given

clear all;
syms t
x = [1  t  t^2  t^3  t^4  t^5];
r1 = subs(x,0);
r2 = subs(diff(x),0);
r3 = subs(diff(diff(x)),0);
r4 = subs(x,10);
r5 = subs(diff(x),10);
r6 = subs(diff(diff(x)),10);
A  = [r1;r2;r3;r4;r5;r6];
B  = [0.5;0;0;0.9;1;0];
c  = inv(A)*B;

%Trajectory equation
tx = c(1) + c(2)*t + c(3)*t^2 +c(4)*t^3 + c(5)*t^4 + c(6)*t^5;
ty = c(1) + c(2)*t + c(3)*t^2 +c(4)*t^3 + c(5)*t^4 + c(6)*t^5;

t = linspace(0,10);
x = subs(tx,t);%linspace(0.5,1.2);
y = subs(ty,t);%linspace(0.5,0.9);

tvx=linspace(0,10);
tvy=linspace(0,10);
vx= subs(diff(tx),tvx);
vy= subs(diff(ty),tvy);
n = size(x,2);

t_int=linspace(0,10);

[T,Y] = ode45(@(t,theta) jvelo(t,theta,tvx,vx,tvy,vy),t_int,[-0.4240 2.4189 -1.2094 ]);
theta1=Y(:,1);
theta2=Y(:,2);
theta3=Y(:,3);

%Calculation of joints
x1=zeros(1,n);
y1=zeros(1,n);
x2=cos(theta1);
y2=sin(theta1);
x4=cos(theta1)+cos(theta1+theta2);
y4=sin(theta1)+sin(theta1+theta2);
x3=x4-0.707;
y3=y4-0.707;

%Plotting of the joint coordinates
for i=1:n
    
    plot([x1(i),x2(i)],[y1(i),y2(i)],'-ro')
    grid on
    hold on
    axis([-2 2 -2 2])
    axis square
    plot([x2(i),x3(i)],[y2(i),y3(i)],'-bo')
                   %axis([-2 2 -2 2])
                   %axis square
    plot([x3(i),x4(i)],[y3(i),y4(i)],'-m')
                   %axis([-2 2 -2 2])
                   %axis square
    pause(0.1)
    hold off
    
end

%End effector trajectory

plot3(t,x4,y4)
title('End-effector trajectory')
grid on