% Function to give tranformation matrix to body referenced frame

function [Rb Rf] = bodyframe(i,R)

if i==1
    Rb = R(:,:,1);
else
Rb= R(:,:,i-1)' * R(:,:,i);
% gives transformation matrix from ith frame to i-1 th frame

end

if i==3
    Rf = 0;
else
    Rf= R(:,:,i)' * R(:,:,i+1);
% gives transformation matrix from i+1 to ith frame
end
end