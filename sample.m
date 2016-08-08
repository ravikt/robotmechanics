

function dx = sample(~,x)
dx    = zeros(2,1);
dx(1) = x(2);
dx(2) = -(12*0.5*9.81)*cos(x(1));

