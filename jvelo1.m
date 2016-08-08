
function dtheta = jvelo(t,theta,tvx,vxe(i,:),tvy,vye(i,:),tome,ome(i,:))

vxe(i,:) = interp1(tvx,vxe(i,:),t); % Interpolate the data set (tvx,vx) at time t
vey(i,:) = interp1(tvy,vey(i,:),t); % Interpolate the data set (tvy,vy) at time t
ome(i,:) = interp1(tome,ome(i,:),t);

dtheta    = zeros(3,1);
l1 = 4;
l2 = 4;
l3 = 4;

    
    %calculation of Jacobian
 j11=-l3*sin(theta(1)+theta(2)+theta(3)) -l2*sin(theta(1)+theta(2)) - l1*sin(theta(1));
 j12=-l2*sin(theta(1)+theta(2)) - l3*sin(theta(1)+theta(2)+theta(3));
 j13=-l3*sin(theta(1)+theta(2)+theta(3));
 j21= l3*cos(theta(1)+theta(2)+theta(3)) + l2*cos(theta(1)+theta(2)) + l1*cos(theta(1));
 j22= l2*cos(theta(1)+theta(2)) + l3*cos(theta(1)+theta(2)+theta(3));
 j23= l3*cos(theta(1)+theta(2)+theta(3));
 j31= 1;
 j32= 1;
 j33= 1;

 J=[j11,j12,j13;j21,j22,j23;j31,j32,j33];
 jinv = pinv(J);
   

 dtheta(1) = jinv(1,1)*vxe(i,:) + jinv(1,2)*vey(i,:);  + jinv(1,3)*ome(i,:);
 dtheta(2) = jinv(2,1)*vex(i,:) + jinv(2,2)*vey(i,:);  + jinv(2,3)*ome(i,:);
 dtheta(3) = jinv(3,1)*vex(i,:) + jinv(3,2)*vey(i,:);  + jinv(3,3)*ome(i,:);