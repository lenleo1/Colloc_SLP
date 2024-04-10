function [f,dfdx,dfdu] = dynamics(x,u,p)
% dx = dynamics(x,u,p)
%
% Computes the dynamics for the simple pendulum
%

theta = x(1,:);
q = x(2,:);

k = p.k;    c = p.c;
dq = -c*q - k*sin(theta) + u;
f = [q; dq];
dfdx = [0, 1; -k*cos(theta), -c];
dfdu = [0; 1];
end