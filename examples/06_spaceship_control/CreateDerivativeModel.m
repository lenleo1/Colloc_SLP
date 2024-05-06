clear;clc
%% Create Symbolic Variables
% State Symbolic Variable
x = sym('x', [5,1]); % px  py  vx  vy Q
% symbolic variable for the controls u
u = sym('u', [2,1]); % ux uy

%% Call the Dynamic Model
% Using the symbolic variables x and u call the dynamic model
% "spaceshipModel". Save the state derivative and the outputs in xdot
xdot = spaceshipModel(x,u);

%% Create Jacobians
Jx = jacobian(xdot,x);
Ju = jacobian(xdot,u);

%% Create MATLAB function
matlabFunction(xdot,Jx,Ju, 'file','spaceshipModelDerivative','vars', {x,u});