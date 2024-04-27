clc; clear;
problem.type = 0; % min tf
% dynamics
% Physical parameters of the pendulum
p.k = 1;  % Normalized gravity constant
p.c = 0.1;  % Normalized damping constant
problem.sys = @(x,u)dynamics(x,u,p);

% collocation grid size
problem.nGrid = 30;
problem.nState = 2; % theta q 
problem.nControl = 1; % u
problem.nOutput = 0; %

% Problem bounds
problem.bounds.Tf_low = 0.5;
problem.bounds.Tf_upp = 2.5;
problem.bounds.x_low = [-2*pi; -inf];
problem.bounds.x_upp = [ 2*pi;  inf];
problem.bounds.xIBC_low = [0; 0];
problem.bounds.xIBC_upp = [0; 0];
problem.bounds.xFBC_low = [pi; 0];
problem.bounds.xFBC_upp = [1.1*pi; 0];
problem.bounds.u_low = -5;
problem.bounds.u_upp =  5;
problem.bounds.u_IBC_low = [];
problem.bounds.u_IBC_upp = [];
problem.bounds.u_FBC_low = [];
problem.bounds.u_FBC_upp = [];
problem.bounds.y_low = [];
problem.bounds.y_upp = [];

% Guess at the initial trajectory
problem.tau_vec = linspace(0, 1, problem.nGrid);
% Guess at the initial trajectory
problem.init.IX = problem.bounds.xIBC_low;
problem.init.FX = problem.bounds.xFBC_low;
problem.init.U = 0;
problem.states = interp1([0;1],[problem.init.IX,problem.init.FX]', problem.tau_vec')';
problem.controls = interp1([0;1], [problem.init.U, problem.init.U]', problem.tau_vec')';
problem.tf = 2;

% scale
problem.scale.tf = 1;
problem.scale.x = [0.3; 0.3];
problem.scale.u = 0.3;
problem.scale.y = [];

%% solver settings
problem.solver.lp_solver = 'matlab_linprog';
problem.solver.iter_key = 6;
problem.solver.iter_max = 25;
problem.solver.delta_max = 10;
problem.solver.delta_s = 0.6;
problem.solver.gamma = 100;
problem.solver.gamma_tr = 0.01; % try 0.001 for better solution
problem.solver.gamma_tr_s = 0.7;
problem.solver.constr_tol = 1e-5;
problem.solver.opt_tol = 1e-4;
problem.solver.step_tol = 1e-5;
%% loop begin
iter_num = 15;
problem.solved = nan;
problem.history.delta_max = nan(iter_num,1);
problem.history.delta_all = nan(iter_num, 1 + problem.nGrid);
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