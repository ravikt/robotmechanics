% Centre of mass of Dual arm Space Robot
% May 1st, 2015

clear all;
close all;
thl1=linspace(pi/4,pi/2+pi/4);
thl2=linspace(-pi/2-pi/3,-pi/2); %All the three angles of left arm
thl3=linspace(-pi/2,-pi/2);
thl=[thl1;thl2;thl3];
% thl=[pi/4 -pi/2-pi/3 -pi/2; pi/2+pi/4 -pi/2 -pi/2];
% thr=[-pi/5 pi/2 pi/2+pi/6; -pi/2-pi/4 pi/2 pi/2];
thr1=linspace(-pi/5,-pi/2-pi/4);
thr2=linspace(pi/2,pi/2);        %All the three angles of right arm   
thr3=linspace(pi/2+pi/6,pi/2);
thr=[thr1;thr2;thr3];


l=[1 1 1]; % Link Length
m=[10 10 10];
x01=0;
y01=0;          
lc=[l(1)/2 l(2)/2 l(3)/2];


% Left arm 

for i=1:3

t=thl(i,:);

lx1(i)=0;
ly1(i)=1;
lx2(i)=lx1(i)+l(1)*cos(t(1));
ly2(i)=ly1(i)+l(1)*sin(t(1));
lx3(i)=lx1(i)+l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2));
ly3(i)=ly1(i)+l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2));
lx4(i)=lx1(i)+l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2)) + l(3)*cos(t(1)+t(2)+t(3));
ly4(i)=ly1(i)+l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2)) + l(3)*sin(t(1)+t(2)+t(3));

lxcm1(i)= (lx2(i)+lx1(i))/2;  %lc(1)*cos(t(1));
lycm1(i)= (ly2(i)+ly1(i))/2;  %lc(1)*sin(t(1));
lxcm2(i)= (lx3(i)+lx2(i))/2;  %l(1)*cos(t(1)) + lc(2)*cos(t(1)+t(2));
lycm2(i)= (ly3(i)+ly2(i))/2;  %l(1)*sin(t(1)) + lc(2)*sin(t(1)+t(2));
lxcm3(i)= (lx4(i)+lx3(i))/2;  %l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2)) + lc(3)*cos(t(1)+t(2)+t(3));
lycm3(i)= (ly4(i)+ly3(i))/2;  %l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2)) + lc(3)*sin(t(1)+t(2)+t(3));
end


% Right arm

for j=1:2
    
t=thr(j,:);

rx1(j)=0;
ry1(j)=-1;
rx2(j)=rx1(j)+l(1)*cos(t(1));
ry2(j)=ry1(j)+l(1)*sin(t(1));
rx3(j)=rx1(j)+l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2));
ry3(j)=ry1(j)+l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2));
rx4(j)=rx1(j)+l(1)*cos(t(1)) + l(2)*cos(t(1)+t(2)) + l(3)*cos(t(1)+t(2)+t(3));
ry4(j)=ry1(j)+l(1)*sin(t(1)) + l(2)*sin(t(1)+t(2)) + l(3)*sin(t(1)+t(2)+t(3));

rxcm1(j)=(rx2(j)+rx1(j))/2; 
rycm1(j)=(ry2(j)+ry1(j))/2; 
rxcm2(j)=(rx3(j)+rx2(j))/2; 
rycm2(j)=(ry3(j)+ry2(j))/2; 
rxcm3(j)=(rx4(j)+rx3(j))/2; 
rycm3(j)=(ry4(j)+ry3(j))/2; 

end


% Calculation of centre of mass of composite system
scraftM = 500;
totalM = scraftM + 2*(m(1)+m(2)+m(3)); % Spacecraft mass plus masses of the two links
xcom = totalM*x01 + (m(1)*(rxcm1(1)+lxcm1(1)) + m(2)*(rxcm2(1)+lxcm2(1)) + m(3)*(rxcm3(1)+lxcm3(1)))/totalM;
ycom = totalM*y01 + (m(1)*(rycm1(1)+lycm1(1)) + m(2)*(rycm2(1)+lycm2(1)) + m(3)*(rycm3(1)+lycm3(1)))/totalM;
% 
x02=(totalM*xcom-(m(1)*(rxcm1(2)+lxcm1(2)) + m(2)*(rxcm2(2)+lxcm2(2)) + m(3)*(rxcm3(2)+lxcm3(2))))/totalM;
y02=(totalM*ycom-(m(1)*(rycm1(2)+lycm1(2)) + m(2)*(rycm2(2)+lycm2(2)) + m(3)*(rycm3(2)+lycm3(2))))/totalM;
