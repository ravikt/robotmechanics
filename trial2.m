% Dynamic Simulation of three link planar manipulator
% using Newton-Euler formulation
% April 27th, 2015

clear all;
%------------------------------------------------------
%Calculation of theta and its derivatives
t= linspace(0,10);

for j=1:100
[thi dthi ddthi] = trialfunc(t(j), 3, 10);
t1(j) = thi(1); %theta1;
t2(j) = thi(2); %theta2; % All the three joint angles
t3(j) = thi(3); %theta3;

t1d(j) = dthi(1);
t2d(j) = dthi(2); % Angular velocities
t3d(j) = dthi(3);

t1dd(j) = ddthi(1);  %diff(t1d)/0.1010;
t2dd(j) = ddthi(2);  %diff(t2d)/0.1010; % Angular accelerations
t3dd(j) = ddthi(3);  %diff(t3d)/0.1010;

end
%---------------------------------------------------------
theta = [t1;t2;t3];

l=[0.3; 0.25; 0.22]; % link lengths
m=[0.5; 0.4; 0.3];   % mass of all the three links

%Inertia tensor
n=3;
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
Icxx(1)=(1/12)*m(1)*.01*.01;   Icyy(1)=(1/12)*m(1)*l(1)*l(1);  Iczz(1)=(1/12)*m(1)*l(1)*l(1); 
Icxx(2)=(1/12)*m(2)*.01*.01;   Icyy(2)=(1/12)*m(2)*l(2)*l(2);  Iczz(2)=(1/12)*m(2)*l(2)*l(2);
Icxx(3)=(1/12)*m(3)*.01*.01;   Icyy(3)=(1/12)*m(3)*l(3)*l(3);  Iczz(3)=(1/12)*m(3)*l(3)*l(3);

I(:,:,1) = [Icxx(1),0,0;0,Icyy(1),0;0,0,Iczz(1)];
I(:,:,2) = [Icxx(2),0,0;0,Icyy(2),0;0,0,Iczz(2)];
I(:,:,3) = [Icxx(3),0,0;0,Icyy(3),0;0,0,Iczz(3)];



ae(:,1)  = [0;0;0];   
f(:,4)   = [0;0;0];  % Initial conditions
tau(:,4) = [0;0;0];

g    = [0; -9.81; 0];

for j = 1:100
 [R] = DH(1,0,0,theta(:,j),3); % Gives all the transformation matrix

 ome(:,1)  = [0;0;t1d(j)]; 
 ome(:,2)  = [0;0;t1d(j)+t2d(j)]; 
 ome(:,3)  = [0;0;t1d(j)+t2d(j)+t3d(j)];

 omed(:,1) = [0;0;t1dd(j)];
 omed(:,2) = [0;0;t1dd(j)+t2dd(j)];
 omed(:,3) = [0;0;t1dd(j)+t2dd(j)+t3dd(j)];


% Forward recursion
 for i=1:3
     
 rici =[l(i)/2;0;0];
 ri   =[l(i);0;0];

 [Rb Rf] = bodyframe(i,R); % Transformation matrix from ith frame to i-1th frame

 ac(:,i) = Rb'*ae(:,i) + cross(omed(:,i),rici) + cross(ome(:,i),cross(ome(:,i),rici)); % acceleration of end of link i
 ae(:,i+1) = Rb'*ae(:,i) + cross(omed(:,i),ri) + cross(ome(:,i),cross(ome(:,i),ri)); % acceleration of end of link i
%ae(:,i) - acceleration of end of link i-1
 gi(:,i) = Rb' *g; % gravity in ith frame

 end
 
 % Backward Recursion
 
 for k=3:-1:1
     
 [Rb Rf] = bodyframe(k,R);
 rici    = [l(k)/2;0;0];
 rci     = [-(l(k)-l(k)/2);0;0];

 f(:,k) = Rf*f(:,k+1) + m(k)*ac(:,k) - m(k)*gi(:,k); % force on link i
 tau(:,k) = Rf*tau(:,k+1) - cross(f(:,k),rici) + cross(Rf*f(:,k+1),rci) + I(:,:,k)*omed(:,k) + cross(ome(:,k),I(:,:,k)*ome(:,k));
 end
 
 TAU1(:,j)= tau(:,1);
 TAU2(:,j)= tau(:,2);
 TAU3(:,j)= tau(:,3);
 
end

figure

subplot(1,3,1)
plot(t,t1)
subplot(1,3,2)
plot(t,t1d)
subplot(1,3,3)
plot(t,t1dd)

figure
plot(t,TAU1(3,:))
title('Torque for all the three links');
xlabel('Time t')
ylabel('Torque tau')
hold on
plot(t,TAU2(3,:),'-g')
hold on
plot(t,TAU3(3,:),'-r')
legend('tau 1','tau 2','tau 3')

figure

animate(l,t1,t2,t3);
