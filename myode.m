

function dydt = myode(t,y,ft,f,gt,g)

% ft = linspace(0,5,25); % Generate t for f
% f = ft.^2 - ft - 3; % Generate f(t)
% gt = linspace(1,6,25); % Generate t for g
% g = 3*sin(gt-0.25); % Generate g(t)

f = interp1(ft,f,t); % Interpolate the data set (ft,f) at time t
g = interp1(gt,g,t); % Interpolate the data set (gt,g) at time t
dydt = -f.*y + g; % Evalute ODE at time t