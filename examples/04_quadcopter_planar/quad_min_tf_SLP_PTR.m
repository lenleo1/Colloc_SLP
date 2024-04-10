clc; clear;
problem.type = 0; % min tf
% dynamics
% Physical parameters of the system
m = 1.5;  % (kg) mass of the quad rotor
g = 9.81;  % (m/s^2) gravity
d = 0.225;  % (m) half-width of quad rotor
J = 0.0179; % (kg m^2) inertia

fMax = 12; % maximum force

% Set up function handles
problem.sys = @(x,u)quad_sys(x,u,m,J,g,d);
% collocation grid size
problem.nGrid = 15;
problem.nState = 6; % [x; y; theta; dx; dy; q] 
problem.nControl = 2; % [f1; f2]
problem.nOutput = 0; %

% Problem bounds
problem.bounds.Tf_low = 0.5;
problem.bounds.Tf_upp = 5;
problem.bounds.x_low = [-inf; -inf; -inf; -inf; -inf; -inf];
problem.bounds.x_upp = [ inf;  inf;  inf;  inf;  inf;  inf];
problem.bounds.xIBC_low = [0; 0; 0; 0; 0; 0];
problem.bounds.xIBC_upp = [0; 0; 0; 0; 0; 0];
problem.bounds.xFBC_low = [5; 0; 0; 0; 0; 0];
problem.bounds.xFBC_upp = [5; 0; 0; 0; 0; 0];
problem.bounds.u_low = [-12; -12];
problem.bounds.u_upp = [12; 12];
problem.bounds.u_IBC_low = [];
problem.bounds.u_IBC_upp = [];
problem.bounds.u_FBC_low = [];
problem.bounds.u_FBC_upp = [];
problem.bounds.output_low = [];
problem.bounds.output_upp = [];

% Guess at the initial trajectory
problem.tau_vec = linspace(0, 1, problem.nGrid);

% Guess at the initial trajectory
problem.init.IX = problem.bounds.xIBC_low;
problem.init.FX = problem.bounds.xFBC_low;
problem.init.U = [11; 11];
problem.states = interp1([0;1],[problem.init.IX,problem.init.FX]', problem.tau_vec')';
problem.controls = interp1([0;1], [problem.init.U, problem.init.U]', problem.tau_vec')';
problem.tf = 2;

% scale
problem.scale.tf = 1;
problem.scale.x = [0.2; 0.3; 5; 0.2; 0.2; 1];
problem.scale.u = [0.1; 0.1];
problem.scale.y = [];

%% solver settings
problem.solver.lp_solver = 'matlab_linprog';
problem.solver.iter_key = 5;
problem.solver.iter_max = 25;
problem.solver.delta_max = 5;
problem.solver.delta_s = 0.7;
problem.solver.gamma = 100;
problem.solver.gamma_tr = 0.01;
problem.solver.gamma_tr_s = 0.7;
problem.solver.constr_tol = 1e-5;
problem.solver.opt_tol = 1e-4;
problem.solver.step_tol = 1e-5;
%% loop begin
iter_num = 25;
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
    if problem.solved == 1
        disp('Local optimality was less than OptimalityTolerance, and maximum constraint violation was less than ConstraintTolerance.');
        break;
    elseif problem.solved == 2
        disp('Step size was less than StepTolerance, and maximum constraint violation was less than ConstraintTolerance.');
        break;
    elseif problem.solved == 0
        disp('Maximum iterations');
        break;
    end
end
%%
plot_problem