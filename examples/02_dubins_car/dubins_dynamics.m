function [Xdot, JX, JU] = dubins_dynamics(X, U)
% x = X(1);
% y = X(2);
alpha = X(3);
v = U(1);
w = U(2);

Xdot = [v*sin(alpha); v*cos(alpha); w];
JX = [0, 0, v*cos(alpha); 
      0, 0, -v*sin(alpha);
      0, 0, 0];
JU = [sin(alpha), 0;
      cos(alpha), 0;
      0, 1];
end