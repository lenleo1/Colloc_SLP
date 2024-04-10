clc; clear;
problem.type = 0; % min tf
% dynamics
problem.sys = @(x,u)acModelDerivative(x,u);

% collocation grid size
problem.nGrid = 31;
problem.nState = 6; % x y z V chi gamma
problem.nControl = 3; % alpha delta_T mu
problem.nOutput = 1; % y

% Problem bounds
problem.bounds.Tf_low = 10;
problem.bounds.Tf_upp = 400;
problem.bounds.x_low = [-5000; -5000; -5000; 20; -2*pi; -pi/6];
problem.bounds.x_upp = [ 5000;  5000;  5000; 120; 2*pi;  pi/6];
problem.bounds.xIBC_low = [0; 0; 0; 100; 0; 0];
problem.bounds.xIBC_upp = [0; 0; 0; 100; 0; 0];
problem.bounds.xFBC_low = [5000; 2000; 0; 100; 0; 0];
problem.bounds.xFBC_upp = [5000; 2000; 0; 100; 0; 0];
problem.bounds.u_low = [0;   0; -pi/6];
problem.bounds.u_upp = [0.6; 1;  pi/6];
problem.bounds.u_IBC_low = [0;   0; -0.01];
problem.bounds.u_IBC_upp = [0.6; 1;  0.01];
problem.bounds.u_FBC_low = [0;   0; -0.01];
problem.bounds.u_FBC_upp = [0.6; 1;  0.01];
problem.bounds.y_low = 0.8;
problem.bounds.y_upp = 1.2;

% Guess at the initial trajectory
problem.tau_vec = linspace(0, 1, problem.nGrid);

load('Initial_guess_N31.mat')
problem.tf = tf0;
problem.controls = controls0;
problem.states = states0;
problem.outputs = nan(problem.nOutput, problem.nGrid);
% compute the output vector
for k = 1:problem.nGrid
    [~,~,~,problem.outputs(:, k),~,~] = problem.sys(problem.states(:,k), problem.controls(:,k));
end

% scale
problem.scale.tf = 0.1;
problem.scale.x = [0.002; 0.001; 0.01; 0.01; 2; 3];
problem.scale.u = [1; 1; 1];
problem.scale.y = 1;

%% solver settings
% problem.lp_solver = 'matlab_linprog';
% problem.solver.iter_key = 6;
% problem.solver.iter_max = 25;
% problem.solver.delta_max = 10;
% problem.solver.delta_s = 0.6;
% problem.solver.gamma = 100;
% problem.solver.gamma_tr = 0.1;
% problem.solver.gamma_tr_s = 0.3;
% problem.solver.eta = 0.01;
% problem.solver.constr_tol = 1e-5;
% problem.solver.opt_tol = 1e-4;
% problem.solver.step_tol = 1e-5;

problem.solver.lp_solver = 'matlab_linprog';
problem.solver.iter_key = 6;
problem.solver.iter_max = 25;
problem.solver.delta_max = 5;
problem.solver.delta_s = 0.6;
problem.solver.gamma = 100;
problem.solver.gamma_tr = 0.1;
problem.solver.gamma_tr_s = 0.7;
problem.solver.constr_tol = 1e-5;
problem.solver.opt_tol = 1e-4;
problem.solver.step_tol = 1e-5;
%% loop begin
iter_num = 25;
problem.solved = 0;
problem.history.delta_max = nan(iter_num,1);
problem.history.step_size = nan(iter_num,1);
problem.history.J = nan(iter_num,1);
problem.history.merit = nan(iter_num,1);
problem.history.rho = nan(iter_num,1);
problem.history.opti_relative = nan(iter_num,1);
problem.history.EQ_vio = nan(iter_num,1);
problem.history.IE_vio = nan(iter_num,1);
tic
for iter = 1:iter_num
    problem.solver.iter = iter;
    problem = CSLP_solver(problem);
    if problem.solved == 1
        disp('Local optimality was less than OptimalityTolerance, and maximum constraint violation was less than ConstraintTolerance.');
        break;
    elseif problem.solved == 2
        disp('Step size was less than StepTolerance, and maximum constraint violation was less than ConstraintTolerance.');
        break;
    end
end
toc
%%
% plot_problem;
