% Euler Lagrange formulation 
% for Three link planar manipulator

l=[0.3; 0.25; 0.22]; % link lengths
lc=[l(1)/2;l(2)/2;l(3)/2];
m=[0.5; 0.4; 0.3];   % mass of all the three links
g = 9.81;

syms t1 t2 t3;
I1=1;I2=2;I3=3;
% Inertia tensor

d11=m(1)*lc(1)^2 + m(2)*(l(1)^2+lc(2)^2+2*l(1)*lc(2)*cos(t2)) + m(3)*...
    (l(1)^2+l(2)^2+lc(3)^2+2*(l(1)*l(2)*cos(t2)+l(2)*lc(3)*cos(t3)+l(1)*lc(3)...
    *cos(t2+t3)))+I1+I2+I3;

d12=m(2)*(lc(2)^2+l(1)*lc(2)*cos(t2)) + m(3)*(l(1)^2+l(2)*lc(2)+l(1)*l(2)...
     *cos(t2)+l(1)*lc(3)*cos(t2+t3)) + l(1)*lc(2)*cos(t2) + lc(2)*lc(3)...
     *cos(t3) + I2+I3;
 
d13=m(3)*(l(1)*lc(1)+l(2)*lc(1)*cos(t2)+lc(3)*lc(1)*cos(t2+t3)) + I3;

d21=d12;

d22=m(2)*lc(2)^2 + m(3)*(l(1)^2+lc(2)^2+2*l(1)*lc(2)*cos(t2)) + I2+I3;

d23=m(3)*(l(1)*lc(1)+lc(2)*lc(1)*cos(t2)) + I3;

d31=d13;

d32=d23;

d33=m(3)*lc(1)^2+I3;