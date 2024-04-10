function [Xdot, JX, JU] = dubins_dynamics_control(X, U)
% x = X(1);
% y = X(2);
alpha = X(3);
% Q = X(4); % integrated control effort
v = U(1);
w = U(2);

Xdot = [v*sin(alpha); v*cos(alpha); w; 0.5*(v^2 + w^2)];
JX = [0, 0, v*cos(alpha), 0; 
      0, 0, -v*sin(alpha), 0;
      0, 0, 0, 0;
      0, 0, 0, 0];
JU = [sin(alpha), 0;
      cos(alpha), 0;
      0, 1;
      v, w];
end