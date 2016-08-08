% Euler-Lagrange formulation
% for Three link planar manipulator
% April 30th, 2015
clear all;
close all;
%Calculation of theta and its derivatives
t= linspace(0,10);

for j=1:100
[thi dthi ddthi] = trialfunc(t(j), 3, 10);
t1(j) = thi(1); %theta1;
t2(j) = thi(2); %theta2; % All the three joint angles
t3(j) = thi(3); %theta3;

dt1(j) = dthi(1);
dt2(j) = dthi(2); % Angular velocities
dt3(j) = dthi(3);

ddt1(j) = ddthi(1);  %diff(t1d)/0.1010;
ddt2(j) = ddthi(2);  %diff(t2d)/0.1010; % Angular accelerations
ddt3(j) = ddthi(3);  %diff(t3d)/0.1010;
end

l=[0.3; 0.25; 0.22]; % link lengths
m=[0.5; 0.4; 0.3];   % mass of all the three links
g = 9.81;

for i=1:100

tau1(i) = (1/3*m(1)*l(1)^2 + m(2)*l(1)^2 + m(3)*l(1)^2 + m(1)*l(1)*l(2)*cos(t2(i))...
    + 2*m(3)*l(1)*l(2)*cos(t2(i)) + m(3)*l(2)^2 + 1/4*m(3)*l(3) + l(2)*l(3)*...
    cos(t3(i)) + l(1)*l(3)*cos(t2(i)+t3(i))) * ddt1(i)...
    + (1/2*m(1)*l(1)*l(2)*cos(t2(i)) + m(3)*l(2)^2 + 1/4*m(3)*l(3) + l(2)*l(3)...
    *cos(t3(i)) + 1/2*l(1)*l(3)*cos(t2(i)+t3(i))) * ddt2(i) + (1/4*m(3)*l(3) + ...
    1/2*l(2)*l(3)*cos(t3(i)) + 1/2*l(1)*l(3)*cos(t1(i)+t2(i)+t3(i))) * ddt3(i) + ...
    ((1/2*m(1)*l(1)*l(2)*sin(t2(i)) + m(3)*l(1)*l(2)*sin(t2(i)))*(2*dt1(i)*dt2(i)+dt2(i)^2))...
    - ((1/2*l(2)*l(3)*sin(t3(i)))*(2*dt1(i)*dt3(i)+2*dt2(i)*dt3(i)+dt3(i)^2)) - ...
    ((1/2*l(1)*l(3)*sin(t2(i)+t3(i)))*(dt2(i)+dt3(i))*(2*dt1(i)+dt2(i)+dt3(i))) + (1/2*m(1)*g*l(1)...
    +m(2)*g*l(1) + m(3)*g*l(1))*cos(t1(i)) + (1/2*m(2)*g*l(2)+m(3)*g*l(2))*...
    cos(t1(i)+t2(i)) + m(3)*g*1/2*l(3)*cos(t1(i)+t2(i)+t3(i));


tau2(i) = (1/3*m(2)*l(2)^2+m(3)*l(2)^2+1/2*m(2)*l(1)*l(2)*cos(t2(i))+1/4*m(3)*l(3)...
    +m(3)*l(1)*l(2)*cos(t2(i))+l(2)*l(3)*cos(t3(i))+1/2*l(1)*l(3)*cos(t2(i)+t3(i))*ddt1(i)...
    +(1/3*m(2)*l(2)^2+m(3)*l(2)^2+1/4*m(3)*l(3)+l(2)*l(3)*cos(t3(i)))*ddt2(i)...
    +(1/4*m(3)*l(3)+1/2*l(2)*l(3))*ddt3(i) - (1/2*m(2)*l(1)*l(2)*sin(t2(i))+...
    m(3)*l(1)*l(2)*sin(t2(i)))*(dt1(i)*dt2(i)) - 1/2*m(3)*l(1)*l(3)*sin(t2(i)+t3(i))...
    *(dt1(i)*dt2(i)+dt3(i)*dt1(i)) + (1/2*m(2)*l(1)*l(2)*sin(t2(i))+m(3)*l(1)*l(2)*sin(t2(i)))*...
    (dt1(i)^2+dt1(i)*dt2(i)) + 1/2*l(1)*l(3)*sin(t2(i)+t3(i))*(dt1(i)^2+dt2(i)*dt1(i)+dt3(i)*dt1(i))...
    + (1/2*m(2)*g*l(2)+m(3)*g*l(2))*cos(t1(i)+t2(i)) + 1/2*m(3)*g*l(3)*cos(t1(i)+t2(i)+t3(i)));


tau3(i) = (1/4*m(3)*l(3)+1/2*m(3)*l(3)*l(2)*cos(t3(i))+1/2*m(3)*l(1)*l(3)*cos(t2(i)+t3(i)))*ddt1(i)...
    +(1/4*m(3)*l(3)+1/2*m(3)*l(3)*l(2))*ddt2(i) + (1/4*m(3)*l(3))*ddt3(i) - 1/2*m(3)*l(3)*l(2)*sin(t3(i))...
    *(dt1(i)*dt3(i)+dt2(i)*dt3(i)) - 1/2*m(3)*l(1)*l(3)*cos(t2(i)+t3(i))*(dt2(i)*dt1(i)+dt3(i)*dt1(i)) + ...
    1/2*m(3)*l(3)*l(2)*sin(t3(i))*(dt1(i)+dt2(i))*(dt1(i)+dt2(i)+dt3(i)) + 1/2*m(3)*l(1)*l(3)*sin(t2(i)+t3(i))...
    *(dt1(i)^2+dt2(i)*dt1(i)+dt3(i)*dt1(i)) + 1/2*m(3)*g*l(3)*cos(t1(i)+t2(i)+t3(i));

end

plot(t,tau1,'-r')
title('Torque for all the three links');
xlabel('Time t')
ylabel('Torque')
hold on
plot(t,tau2)
hold on
plot(t,tau3,'-m')
legend('tau 1','tau 2','tau 3')

figure

animate(l,t1,t2,t3);
