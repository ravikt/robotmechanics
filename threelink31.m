%Simulation of three link planar manipulator
%Involves calculation of Jacobian for each configuration
%The calculation is done using ode45
%A function jvelo is defined to perform the integration.
%Initial configuration for phi = pi/4 is given

clear all;
close all;
syms t
xi   = input('Enter initial coordinates ');
xf   = input('Enter final coordinates   ');
phi = input('Enter the end effector oreintation ');
x = [1  t  t^2  t^3  t^4  t^5];
r1 = subs(x,0);
r2 = subs(diff(x),0);
r3 = subs(diff(diff(x)),0);
r4 = subs(x,10);
r5 = subs(diff(x),10);
r6 = subs(diff(diff(x)),10);
A  = [r1;r2;r3;r4;r5;r6];

l1 = 2;        % Link lengths
l2 = 2;
l3 = 2;
% Calculation of joint variables for initial coordinates
xe =   xi(1);
ye =   xi(2);

k1 = xe - l3*cos(phi);
k2 = ye - l3*sin(phi);

c2 = (k1^2 + k2^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = sqrt(1 - c2^2);

t2 = atan2(s2,c2);

alpha = atan2(k2,k1);
beta  = atan2(l2*sin(t2),l1+l2*cos(t2));
t1= alpha - beta ;
t3= phi - (t1+t2);
%-------------------------------------------
B  = [xi(1);0;0;xf(1);1;0]; % x component of initial and final conditions.
D  = [xi(2);0;0;xf(2);1;0]; % y component of initial anf final conditions. 
E  = [phi;0;0;phi;1;0];% conditioins for angular velocity.
c  = pinv(A)*B;
d  = pinv(A)*D;
e  = pinv(A)*E;

%Trajectory equation
tx  = c(1) + c(2)*t + c(3)*t^2 +c(4)*t^3 + c(5)*t^4 + c(6)*t^5;
ty  = d(1) + d(2)*t + d(3)*t^2 +d(4)*t^3 + d(5)*t^4 + d(6)*t^5;
tphi= e(1) + e(2)*t + e(3)*t^2 +e(4)*t^3 + e(5)*t^4 + e(6)*t^5;

t = linspace(0,10);
x = subs(tx,t);%linspace(0.5,1.2);
y = subs(ty,t);%linspace(0.5,0.9);

tvx  = linspace(0,10);
tvy  = linspace(0,10);
tome = linspace(0,10);

vx  = subs(diff(tx),tvx);
vy  = subs(diff(ty),tvy);
ome = subs(diff(tphi),tome);

% n = size(x,2);

t_int=[0 10];

[T,Y] = ode45(@(t,theta) jvelo(t,theta,tvx,vx,tvy,vy,tome,ome),t_int,[t1 t2 t3]);
theta1=Y(:,1);
theta2=Y(:,2);
theta3=Y(:,3);

n = length(T);


%Calculation of joints
n=length(T);
x1=zeros(1,n);
y1=zeros(1,n);
x2=l1*cos(theta1);
y2=l1*sin(theta1);
x3=l1*cos(theta1) + l2*cos(theta1+theta2);
y3=l1*sin(theta1) + l2*sin(theta1+theta2);
x4=l1*cos(theta1) + l2*cos(theta1+theta2) + l3*cos(theta1+theta2+theta3);
y4=l1*sin(theta1) + l3*sin(theta1+theta2) + l3*sin(theta1+theta2+theta3);


%Plotting of the joint coordinates
for i=1:n
    
    plot([x1(i),x2(i)],[y1(i),y2(i)],'-ro')
    grid on
    hold on
    axis([-2*l1 2*l1 -2*l1 2*l1])
    axis square
    plot([x2(i),x3(i)],[y2(i),y3(i)],'-bo')
    plot([x3(i),x4(i)],[y3(i),y4(i)],'-m')
    pause(0.1)
    hold off
    
end

%End effector trajectory

% plot3(T,x4,y4)
% title('End-effector trajectory')
% grid on