%Newton-Euler formulation
%The function gives homogenous transformation matrix using DH parameters

function [R] = DH(a,d,al,th,dof)
B=eye(3,3);
R=zeros(3,3,dof);
for i=1:dof
A =  [cos(th(i)),-sin(th(i))*cos(al),sin(th(i))*sin(al);sin(th(i)),cos(th(i))*cos(al),-cos(th(i))*sin(al);0,sin(al),cos(al)];


B = B*A;

R(:,:,i)=B;
% R=B
end

end