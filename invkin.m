% Solution of inverse kinematics problem for 
% Threelink (RRR) planar manipulator
% using Geometric approach
% April 15th,2015

clear all;
close all;
syms t
x = [1  t  t^2  t^3  t^4  t^5];
xi   = input('Enter initial coordinates ');
xf   = input('Enter final coordinates   ');
phi  = input('Enter the end effector oreintation ');
tint = input('Enter the time interval ');
ti   = tint(1);
tf   = tint(2);
r1 = subs(x,ti);
r2 = subs(diff(x),ti);
r3 = subs(diff(diff(x)),ti);
r4 = subs(x,tf);
r5 = subs(diff(x),tf);
r6 = subs(diff(diff(x)),tf);
A  = [r1;r2;r3;r4;r5;r6];
B  = [xi(1);0;0;xf(1);1;0]; % for x component of initial and final conditions
C  = [xi(2);0;0;xf(2);1;0]; % for y component of initial and final condition 
c  = inv(A)*B;
b  = inv(A)*C;

l1 = 2;
l2 = 2;
l3 = 2;

tx = c(1) + c(2)*t + c(3)*t^2 +c(4)*t^3 + c(5)*t^4 + c(6)*t^5;
ty = b(1) + b(2)*t + b(3)*t^2 +b(4)*t^3 + b(5)*t^4 + b(6)*t^5;

t = linspace(ti,tf);
xe = subs(tx,t);%linspace(0.5,1.2);
ye = subs(ty,t);%linspace(0.5,0.9);
% vx= subs(diff(tx),t);
% vy= subs(diff(ty),t);
n = size(xe,2);
theta1 = zeros(1,n);
theta2 = zeros(1,n);
theta3 = zeros(1,n);

for i = 1:n

k1 = xe(i) - l3*cos(phi);
k2 = ye(i) - l3*sin(phi);

c2 = (k1^2 + k2^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = sqrt(1 - c2^2);

theta2(i) = atan2(s2,c2);

alpha = atan2(k2,k1);
beta  = atan2(l2*sin(theta2(i)),l1+l2*cos(theta2(i)));
theta1(i) = alpha - beta ;

theta3(i) = phi - (theta1(i)+theta2(i));

end

x1=zeros(1,n);
y1=zeros(1,n);
x2=l1*cos(theta1);
y2=l1*sin(theta1);
x3=l1*cos(theta1) + l2*cos(theta1+theta2);
y3=l1*sin(theta1) + l2*sin(theta1+theta2);
x4=l1*cos(theta1) + l2*cos(theta1+theta2) + l3*cos(theta1+theta2+theta3);
y4=l1*sin(theta1) + l3*sin(theta1+theta2) + l3*sin(theta1+theta2+theta3);

for i=1:n
   
    plot([x1(i),x2(i)],[y1(i),y2(i)],'-ro')
    hold on
    axis([-3*l1 3*l1 -3*l1 3*l1])
    axis square
    plot([x2(i),x3(i)],[y2(i),y3(i)],'-bo')
                   
    plot([x3(i),x4(i)],[y3(i),y4(i)],'-m')
                   
    pause(0.1)
    hold off
     
 end