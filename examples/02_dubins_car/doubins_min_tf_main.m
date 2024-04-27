clc; clear;
problem.type = 0; % min tf
% dynamics
problem.sys = @dubins_dynamics;

% collocation grid size
problem.nGrid = 21;
problem.nState = 3; % x v
problem.nControl = 2; % u
problem.nOutput = 0; % y

% Problem bounds
problem.bounds.Tf_low = 2;
problem.bounds.Tf_upp = 10;
problem.bounds.x_low = [-4;  -4;  -pi];
problem.bounds.x_upp = [ 4;   4;   pi];
problem.bounds.xIBC_low = [-3; -2.5; pi/2];
problem.bounds.xIBC_upp = [-3; -2.5; pi/2];
problem.bounds.xFBC_low = [3; 2.5; 0];
problem.bounds.xFBC_upp = [3; 2.5; 0];
problem.bounds.u_low = [0; -0.5];
problem.bounds.u_upp = [1;  0.5];
problem.bounds.u_IBC_low = [];
problem.bounds.u_IBC_upp = [];
problem.bounds.u_FBC_low = [];
problem.bounds.u_FBC_upp = [];

% Guess at the initial trajectory
problem.tau_vec = linspace(0, 1, problem.nGrid);

problem.tf = 10;
problem.controls = repmat([0.5; 0], 1, problem.nGrid);
problem.states = GenericRK('4thOrder', problem.sys, problem.tf*problem.tau_vec,...
    problem.bounds.xIBC_low , problem.controls, 0);
problem.states(:,end) = problem.bounds.xFBC_low;

% scale
problem.scale.tf = 1;
problem.scale.x = [0.5; 0.5; 1];
problem.scale.u = [1; 1];
problem.scale.y = [];

%% solver settings
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
iter_num = 15;
problem.solved = nan;
problem.history.delta_max = nan(iter_num,1);
problem.history.step_size = nan(iter_num,1);
problem.history.J = nan(iter_num,1);
problem.history.merit = nan(iter_num,1);
problem.history.rho = nan(iter_num,1);
problem.history.opti_relative = nan(iter_num,1);
problem.history.EQ_vio = nan(iter_num,1);

for iter = 1:iter_num
    problem.solver.iter = iter;
    problem = CSLP_solver(problem);
    if problem.solved == 0
        disp('Maximum iterations');
        break;
    elseif problem.solved == 1
        disp('Local optimality was less than OptimalityTolerance, and maximum constraint violation was less than ConstraintTolerance.');
        break;
    elseif problem.solved == 2
        disp('Step size was less than StepTolerance, and maximum constraint violation was less than ConstraintTolerance.');
        break;
    elseif problem.solved == 3
        disp('Step size was less than StepTolerance, but maximum constraint violation exceeds ConstraintTolerance.');
        break;
    else
        disp('continue with the iteration...');
    end
end
%%
plot_problem