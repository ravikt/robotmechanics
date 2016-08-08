% Path Planning
% Generates cubic trajectory via specified points
% End effector passes through [0,0]-[0.3,0.3]-[0.5,0.5]-[0.7,0.7]
% Velocity v(0) = 0, v(3) = 1, v(6) = 2, v(9) = 3
% Angular velocity omega(0) = 0, omega(3) = 0.5
% omega(6) = 1, omega(9) = 2
% April 13th, 2015

clear all;
close all;
syms t
x  = [1  t  t^2  t^3];
phi = pi/4;       % User defined end effector orientation

Bx  = [0,0,0.3,1;0.3,1,0.5,2;0.5,2,0.7,3];
By  = [0,0,0.3,1;0.3,1,0.5,2;0.5,2,0.7,3];
Bo  = [phi,0,phi,0.5;phi,0.5,phi,1;phi,1,phi,2];

xe = zeros(3,100);
ye = zeros(3,100);
phe= zeros(3,100);
tint=zeros(3,100);
t0 = [0,3,6];
tf = [3,6,9];

vxe = zeros(3,100); 
vye = zeros(3,100);
omega = zeros(3,100); % angular velocity of end effector

for i=1:3

    
r1 = subs(x,t0(i));
r2 = subs(diff(x),t0(i));
r3 = subs(x,tf(i));
r4 = subs(diff(x),tf(i));
A  = [r1;r2;r3;r4];

b  = inv(A)*Bx(i,:)';
c  = inv(A)*By(i,:)'; 
d  = inv(A)*Bo(i,:)';

% %Trajectory equation
tx(i)   = b(1) + b(2)*t + b(3)*t^2 + b(4)*t^3;
ty(i)   = c(1) + c(2)*t + c(3)*t^2 + c(4)*t^3;
tphi(i) = d(1) + d(2)*t + d(3)*t^2 + d(4)*t^3;

tint(i,:) = linspace(t0(i),tf(i)); % Generates time elements for each trajectory

xe(i,:) = subs(tx(i),tint(i,:));   % x coordinate of end effector
ye(i,:) = subs(ty(i),tint(i,:));   % y coordinate of end effector
phe(i,:)= subs(tphi(i),tint(i,:)); % end effector orientation

vxe(i,:)   = subs(diff(tx(i)),tint(i,:));   
vye(i,:)   = subs(diff(ty(i)),tint(i,:));
omega(i,:) = subs(diff(tphi(i)),tint(i,:));
end

xef  =  [xe(1,:) xe(2,:) xe(3,:)];
yef  =  [ye(1,:) ye(2,:) ye(3,:)];
time =  [tint(1,:) tint(2,:) tint(3,:)];

vex  =  [vxe(1,:) vxe(2,:) vxe(3,:)]; % x component of end effector velocity
vey  =  [vye(1,:) vye(2,:) vye(3,:)]; % y component of end effector velocity
ome  =  [omega(1,:) omega(2,:) omega(3,:)]; % Angular velocity of end effector


n = size(vex,2);
%Calculation of Jacobian and theta values
theta1 = zeros(1,n);
theta2 = zeros(1,n);
theta3 = zeros(1,n);


theta1(1) = -3.4034;
theta2(1) =  2.0944;
theta3(1) =  2.0944;
 
l1 = 4;
l2 = 4;            % Link length of manipulator
l3 = 4;

 for i=1:n
    
 j11=-l3*sin(theta1(i)+theta2(i)+theta3(i)) -l2*sin(theta1(i)+theta2(i)) - l1*sin(theta1(i));
 j12=-l2*sin(theta1(i)+theta2(i)) - l3*sin(theta1(i)+theta2(i)+theta3(i));
 j13=-l3*sin(theta1(i)+theta2(i)+theta3(i));
 j21= l3*cos(theta1(i)+theta2(i)+theta3(i)) + l2*cos(theta1(i)+theta2(i)) + l1*cos(theta1(i));
 j22= l2*cos(theta1(i)+theta2(i)) + l3*cos(theta1(i)+theta2(i)+theta3(i));
 j23= l3*cos(theta1(i)+theta2(i)+theta3(i));
 j31= 1;
 j32= 1;
 j33= 1;

 v=[vex(i);vey(i);ome(i)];
 J=[j11,j12,j13;j21,j22,j23;j31,j32,j33];
 thetad = pinv(J)*v;
    t1d(i) = thetad(1);
    t2d(i) = thetad(2); % Angular velocities
    t3d(i) = thetad(3);
 theta1(i+1) = theta1(i) + (0.0303*thetad(1));
 theta2(i+1) = theta2(i) + (0.0303*thetad(2));
 theta3(i+1) = theta3(i) + (0.0303*thetad(3));
    
 end

x1=zeros(1,n);
y1=zeros(1,n);
x2=l1*cos(theta1);
y2=l1*sin(theta1);
x3=l1*cos(theta1) + l2*cos(theta1+theta2);
y3=l1*sin(theta1) + l2*sin(theta1+theta2);
x4=l1*cos(theta1) + l2*cos(theta1+theta2) + l3*cos(theta1+theta2+theta3);
y4=l1*sin(theta1) + l2*sin(theta1+theta2) + l3*sin(theta1+theta2+theta3);

% 
% for i=1:n
%     
%     plot([x1(i),x2(i)],[y1(i),y2(i)],'-ro')
%     grid on
%     hold on
%     axis([-2.5*l1 2.5*l1 -2.5*l1 2.5*l1])
%     axis square
%     plot([x2(i),x3(i)],[y2(i),y3(i)],'-bo')
%                    
%     plot([x3(i),x4(i)],[y3(i),y4(i)],'-m')
%                    
%     pause(0.1)
%     hold off
%    
%     
% end

subplot(3,1,1)
plot(time,x4(1:300))
title('X cooridnate of end effector trajectory')
subplot(3,1,2)
plot(time,y4(1:300))
title('Y cooridnate of end effector trajectory')
subplot(3,1,3)
plot(time,ome)
title('Angular velocity of end effector')

figure
plot(time,theta1(1:300))
title('Joint variables')
hold on
plot(time,theta2(1:300),'-r')
hold on
plot(time,theta3(1:300),'-m')
legend('theta1','theta2','theta3')