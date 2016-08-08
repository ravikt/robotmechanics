% ReDySim trajectory module. The desired indpendent joint trejectories are 
% enterd here
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [thi dthi ddthi]= trialfunc(tim, dof, Tp)
%Enter trejectories here
%Constant angular velocity
% Initial and final joint variable



thin=[0 0 0];
thf=[pi pi pi ];
for i=1:dof
    thi(i)=thin(i)+((thf(i)-thin(i))/Tp)*(tim-(Tp/(2*pi))*sin((2*pi/Tp)*tim));
    dthi(i)=((thf(i)-thin(i))/Tp)*(1-cos((2*pi/Tp)*tim));
    ddthi(i)=(2*pi*(thf(i)-thin(i))/(Tp*Tp))*sin((2*pi/Tp)*tim);
end
