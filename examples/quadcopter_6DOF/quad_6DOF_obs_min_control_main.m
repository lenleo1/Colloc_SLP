clc; clear;
problem.type = 1; % min control effort (lagrange type)
problem.sys = @(x,u)quad_6dof_obs_sys(x,u); % system dynamics

% problem dimensions
problem.nGrid = 21; % collocation grid size
problem.nState = 13; %[x, y, z, pitch, roll, yaw, dx, dy, dz, dpitch, droll, dyaw, lag_int]
problem.nControl = 4; % [u1; u2; u3; u4]
problem.nOutput = 1; % y = obstacle_cons

% Problem bounds
problem.bounds.Tf_low = 1;
problem.bounds.Tf_upp = 10;
problem.bounds.x_low = [-100*ones(12, 1); 0];
problem.bounds.x_upp = [100*ones(12, 1); 10];
problem.bounds.xIBC_low = [zeros(12,1); 0];
problem.bounds.xIBC_upp = [zeros(12,1); 0];
problem.bounds.xFBC_low = [10; zeros(11,1); 0.5];
problem.bounds.xFBC_upp = [10; zeros(11,1); 10];
problem.bounds.u_low = [0; 0; 0; 0];
problem.bounds.u_upp = [1; 1; 1; 1];
problem.bounds.u_IBC_low = [];
problem.bounds.u_IBC_upp = [];
problem.bounds.u_FBC_low = [];
problem.bounds.u_FBC_upp = [];
problem.bounds.y_low = 0;
problem.bounds.y_upp = inf;

% Guess at the initial trajectory
problem.tau_vec = linspace(0, 1, problem.nGrid);

problem.tf = 5;
problem.states = interp1([0;1],[(problem.bounds.xIBC_low+problem.bounds.xIBC_upp)/2,...
    (problem.bounds.xFBC_low+problem.bounds.xFBC_upp)/2]', problem.tau_vec')';
u_init = ones(4,1);
problem.controls = interp1([0;1], [u_init, u_init]', problem.tau_vec')';
% compute the output vector
problem.outputs = nan(problem.nOutput, problem.nGrid);
for k = 1:problem.nGrid
    [~,~,~,problem.outputs(:, k),~,~] = problem.sys(problem.states(:,k), problem.controls(:,k));
end

%% scale
problem.scale.tf = 2;
problem.scale.x = [0.1; 1; 1; 10;  1;  5;    0.1; 1;  1;  1; 0.2; 1; 1];
%                 [x,  y, z, pitch,roll,yaw, dx,  dy, dz, q,  p,  r; lag_int]
problem.scale.u = ones(4,1);
problem.scale.y = 1;

%% solver settings
problem.solver.lp_solver = 'gorubi_linprog';
problem.solver.iter_key = 6;
problem.solver.iter_max = 25;
problem.solver.delta_max = 5;
problem.solver.delta_s = 0.7;
problem.solver.gamma = 100;
problem.solver.gamma_tr = 0.5;
problem.solver.gamma_tr_s = 0.8;
problem.solver.constr_tol = 1e-6;
problem.solver.opt_tol = 1e-5;
problem.solver.step_tol = 1e-4;
%% loop begin
iter_num = 24;
problem.history.delta_max = nan(iter_num,1);
problem.history.step_size = nan(iter_num,1);
problem.history.J = nan(iter_num,1);
problem.history.merit = nan(iter_num,1);
problem.history.rho = nan(iter_num,1);
problem.history.opti_relative = nan(iter_num,1);
problem.history.EQ_vio = nan(iter_num,1);
problem.history.IE_vio = nan(iter_num,1);

problem.solved = nan;
tic
for iter = 1:iter_num
    problem.solver.iter = iter;
    problem = CSLP_solver(problem);
    if problem.solved == 0
        disp('Maximum iterations reached!');
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
toc
%%
% plot_problem;
