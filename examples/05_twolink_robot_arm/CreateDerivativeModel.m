clear;clc
%% Create Symbolic Variables
% State Symbolic Variable
x = sym('x', [4,1]); % omega_alpha  omega_beta  theta  alpha
% symbolic variable for the controls u
u = sym('u', [2,1]); % u1 u2

%% Call the Dynamic Model
% Using the symbolic variables x and u call the dynamic model
% "rocketModel". Save the state derivative and the outputs in xdot and y
xdot = robotModel(x,u);

%% Create Jacobians
Jx = jacobian(xdot,x);
Ju = jacobian(xdot,u);

%% Create MATLAB function
matlabFunction(xdot,Jx,Ju, 'file','robotModelDerivative','vars', {x,u});