% Simulation of three link planar manipulator
% Involves calculation of Jacobian for each configuration
% Initial velocity = 0, final velocity = 1
% April 13th, 2015

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
% 
l1 = 2.5;        % Link lengths
l2 = 2.5;
l3 = 2.5;
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

t  = linspace(0,10);

x  = subs(tx,t);
y  = subs(ty,t);
phi1= subs(tphi,t);
vx   = subs(diff(tx),t);
vy   = subs(diff(ty),t);
omega= subs(diff(phi1),t);
n = size(x,2);
theta1 = zeros(1,n);
theta2 = zeros(1,n);
theta3 = zeros(1,n);


theta1(1)= t1;
theta2(1)= t2; %Initial joint angles
theta3(1)= t3;



for i=1:n-1
    
 %calculation of Jacobian
 j11=-l3*sin(theta1(i)+theta2(i)+theta3(i)) -l2*sin(theta1(i)+theta2(i)) - l1*sin(theta1(i));
 j12=-l2*sin(theta1(i)+theta2(i)) - l3*sin(theta1(i)+theta2(i)+theta3(i));
 j13=-l3*sin(theta1(i)+theta2(i)+theta3(i));
 j21= l3*cos(theta1(i)+theta2(i)+theta3(i)) + l2*cos(theta1(i)+theta2(i)) + l1*cos(theta1(i));
 j22= l2*cos(theta1(i)+theta2(i)) + l3*cos(theta1(i)+theta2(i)+theta3(i));
 j23= l3*cos(theta1(i)+theta2(i)+theta3(i));
 j31= 1;
 j32= 1;
 j33= 1;

 v=[vx(i);vy(i);omega(i)];
 J=[j11,j12,j13;j21,j22,j23;j31,j32,j33];
 thetad = pinv(J)*v;
 
 %----------------------
%  
 t1d(i) = thetad(1);
 t2d(i) = thetad(2); % Angular velocities
 t3d(i) = thetad(3);
 %-------------------------
 theta1(i+1) = theta1(i) + (0.1010*thetad(1));
 theta2(i+1) = theta2(i) + (0.1010*thetad(2));
 theta3(i+1) = theta3(i) + (0.1010*thetad(3));
    
end
%Calculation of joints
x1=zeros(1,n);
y1=zeros(1,n);
x2=l1*cos(theta1);
y2=l1*sin(theta1);
x3=l1*cos(theta1) + l2*cos(theta1+theta2);
y3=l1*sin(theta1) + l2*sin(theta1+theta2);
x4=l1*cos(theta1) + l2*cos(theta1+theta2) + l3*cos(theta1+theta2+theta3);
y4=l1*sin(theta1) + l3*sin(theta1+theta2) + l3*sin(theta1+theta2+theta3);
%Plotting of the joint coordinates
% for i=1:n
%     
%     
%     grid on
%     plot([x1(i),x2(i)],[y1(i),y2(i)],'-ro')
%     title('Simulation of three link manipulator')
%      hold on
%      axis([-2.5*l1 2.5*l1 -2.5*l1 2.5*l1])
%      axis square
%     
%     plot([x2(i),x3(i)],[y2(i),y3(i)],'-bo')
%     plot([x3(i),x4(i)],[y3(i),y4(i)],'-m')
%     pause(0.1)
%     hold off
%     
% end



%------------------------------------------------------------------------
% NEWTON-EULER FORMULATION
% Calculation of force and torque cetor for each configuration
% while following a qunitic trajectory
% April 20th, 2015
m1 = 1; 
m2 = 1;  % Mass of the three links
m3 = 1;

I1 = (1/12)*m1*(l1/2)^2;
I2 = (1/12)*m2*(l2/2)^2;
I3 = (1/12)*m3*(l3/3)^2;

lc1 = [l1/2;0;0];
lc2 = [l2/2;0;0]; % Vector from joint i-1 to centre of mass of link i
lc3 = [l3/2;0;0]; % for all the three links

t1 =  theta1;
t2 =  theta2; % All the three joint angles
t3 =  theta3;

t1dd = diff(t1d)/0.1010;
t2dd = diff(t2d)/0.1010; % Angular accelerations
t3dd = diff(t3d)/0.1010;

F1 = zeros(3,n-2);
F2 = zeros(3,n-2);
F3 = zeros(3,n-2);

TAU1 = zeros(3,n-2);
TAU2 = zeros(3,n-2);
TAU3 = zeros(3,n-2);

for j=1:n-2
% t1 =  theta1(j);
% t2 =  theta2(j); % All the three joint angles
% t3 =  theta3(j);

g1 = [sin(t1(j));-cos(t1(j));0];
g2 = [sin(t1(j)+t2(j));-cos(t1(j)+t2(j));0];
g3 = [sin(t1(j)+t2(j)+t3(j));-cos(t1(j)+t2(j)+t3(j));0];
      
% FORWARD RECURSION

%---Link1---

ac1 = [-lc1(1)*t1d(j)^2;lc1(1)*t1dd(j);0]; % acceleration of centre of mass of link 1

ae1 = [-l1*t1d(j)^2;l1*t1dd(j);0]; % acceleration of end of link 1
%---Link2---

A = -l1*t1d(j)^2*cos(t2(j)) + l1*t1dd(j)*sin(t2(j)) - (t1d(j)+t2d(j))^2 * lc2(1);
B = -l1*t1d(j)^2*sin(t2(j)) + l1*t1dd(j)*cos(t2(j)) + (t1dd(j)+t2dd(j)) * lc2(1);

ac2 = [A;B;0]; % acceleration of centre of mass of link 2

%---Link3---
A1 = -l1*t1d(j)^2*cos(t2(j)) + l1*t1dd(j)*sin(t2(j)) - (t1d(j)+t2d(j))^2 * l2;
B1 = -l1*t1d(j)^2*sin(t2(j)) + l1*t1dd(j)*cos(t2(j)) + (t1dd(j)+t2dd(j)) * l2;

ae2= [A1;B1;0]; % acceleration of end of link2

A2 = cos(t3(j))*A1 + sin(t3(j))*B1 - (t1d(j)+t2d(j)+t3d(j))^2 * lc3(1) ;
B2 = -sin(t3(j))*A1 + cos(t3(j))*B1 + (t1d(j)+t2d(j)+t3d(j))^2 * lc3(1) ;

ac3= [A2;B2;0]; % acceleratioin of centre of mass of link 3

% BACKWARD RECURSION

alpha1 = t1dd(j);
alpha2 = t1dd(j)+t2dd(j);
alpha3 = t1dd(j)+t2dd(j)+t3dd(j);

a1=[0;0;alpha1];
a2=[0;0;alpha2];
a3=[0;0;alpha3];

r3c2 = [-(l2-lc2(1));0;0];
r2c1 = [-(l1-lc1(1));0;0];

%---Link3---

f3 = m3*ac3 - m3*g3; % force on link 3

F3(:,j) = f3;

tau3 = I3*a3 - cross(f3,lc3); %torque on link 3

TAU3(:,j) = tau3;

%---Link2---

A3 = [cos(t3(j))*f3(1)-sin(t3(j))*f3(2);sin(t3(j))*f3(1)-cos(t3(j))*f3(2);f3(3)]; % f3 in frame2

f2 = A3 - m2*ac2 - m2*g2; % force on link 2

F2(:,j) = f2;

B3 = [cos(t3(j))*tau3(1)-sin(t3(j))*tau3(2);sin(t3(j))*tau3(1)-cos(t3(j))*tau3(2);tau3(3)]; % tau3 in frame 2

tau2 = B3 - cross(f2,lc2) + cross(A3,r3c2) + I2*a2; % torque on link2

TAU2(:,j) = tau2;


%---Link1---

A4 = [cos(t2(j))*f2(1)-sin(t2(j))*f2(2);sin(t2(j))*f2(1)-cos(t2(j))*f2(2);f2(3)]; % f2 in frame 1

f1 = A4 + m1*ac1 - m1*g1; % force on link 1

F1(:,j) = f1;

B4 = [cos(t2(j))*tau2(1)-sin(t2(j))*tau2(2);sin(t2(j))*tau2(1)-cos(t2(j))*tau2(2);tau2(3)]; % tau2 in frame 1

tau1 = B4 - cross(f1,lc1) + cross(A4,r2c1) + I1*a1; % torque on link1

TAU1(:,j) = tau1;

end

% Plot results
subplot(2,1,1)
plot(t,x4)
title('X coordinate of end effector trajectory')
subplot(2,1,2)
plot(t,y4)
title('Y coordinate of end effector trajectory')

figure
plot(t(1:98),TAU1(3,:))
title('Torque for all the three links');
xlabel('Time t')
ylabel('Torque tau')
hold on
plot(t(1:98),TAU2(3,:),'-r')
hold on
plot(t(1:98),TAU3(3,:),'-g')
legend('tau 1','tau 2','tau 3')