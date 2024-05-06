function [dx] = spaceshipModel(x,u)
% Optimal Control for a Spaceship - Dynamics
% % This implementation was adapted from
% Knauer M., Büskens C. (2019) Real-Time Optimal Control Using TransWORHP and WORHP Zen. 
% In: Fasano G., Pintér J. (eds) Modeling and Optimization in Space Engineering.
% Springer Optimization and Its Applications, vol 144. Springer, Cham
%
% Inputs:
%    x  - state vector
%    u  - input
%    data - additional data   
% Output:
%    dx - time derivative of x


px = x(1);py = x(2);vx = x(3);vy = x(4);
ux = u(1);uy = u(2);

mu = 0.01;
x1 = -mu;
x2 = 1 - mu;
r1 = sqrt((px-x1).^2 + py.^2);
r2 = sqrt((px-x2).^2 + py.^2);

px_dot = vx;
py_dot = vy;
vx_dot = 2.*vy + px - (1-mu).*(px-x1)./(r1.^3) - mu.*(px-x2)./(r2.^3) + ux;
vy_dot = -2.*vx + py - (1-mu).*py./(r1.^3) - mu.*py./(r2.^3) + uy;
Q_dot = 1 + ux.^2 + uy.^2; % lagrange term
dx = [px_dot; py_dot; vx_dot; vy_dot; Q_dot];
end