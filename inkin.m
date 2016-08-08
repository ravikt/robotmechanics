% Inverse kinematic soultion using geometric approach
% Works only for single point
% for the purpose of generating joint variables for given coordinate.
% function [theta1 theta2 theta3] = inkin(xe,ye)
clear all;
close all;

xe =   0;
ye =   2;

phi = pi/3;

l1 = 1;
l2 = 1;
l3 = 1;

k1 = xe - l3*cos(phi);
k2 = ye - l3*sin(phi);

c2 = (k1^2 + k2^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = sqrt(1 - c2^2);

theta2 = atan2(s2,c2);

alpha = atan2(k2,k1);
beta  = atan2(l2*sin(theta2),l1+l2*cos(theta2));
theta1= alpha - beta ;
theta3= phi - (theta1+theta2);

x1=-1;
y1=0;
x2=l1*cos(theta1);
y2=l1*sin(theta1);
x3=l1*cos(theta1) + l2*cos(theta1+theta2);
y3=l1*sin(theta1) + l2*sin(theta1+theta2);
x4=l1*cos(theta1) + l2*cos(theta1+theta2) + l3*cos(theta1+theta2+theta3);
y4=l1*sin(theta1) + l2*sin(theta1+theta2) + l3*sin(theta1+theta2+theta3);


plot([x1,x2],[y1,y2],'-ro')
    hold on
    axis([-2*l1 2*l1 -2*l1 2*l1])
    
plot([x2,x3],[y2,y3],'-bo')
                   
plot([x3,x4],[y3,y4],'-m')
    hold off
   
theta1
theta2
theta3

