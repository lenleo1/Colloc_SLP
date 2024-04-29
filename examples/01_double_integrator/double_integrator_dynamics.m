function [Xdot, JX, JU] = double_integrator_dynamics(X, U)
x = X(1);
v = X(2);
xdot = v;
vdot = U;
Xdot = [xdot; vdot];
JX = [0, 1; 
      0, 0];
JU = [0; 1];
end