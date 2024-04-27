function problem = CSLP_solver(problem_in)
% Collocation based Successive Linear Programming (CSLP) Solver.
% This function solve the trajectory optimization problem using the
% collocation based successive linear programming method.

% Version 1.0
% usage: problem = CSLP_solver(problem_in)
% input: problem_in    - the problem to be solved
% output: problem      - the solved problem
% Porblems can be divided into two types:
% type 0 - minimum time,
% type 1 - minimum lagrange integration
% type 2 - tbd
% Written by:    Zhidong Lu
% e-mail:        zhidong.lu@tum.de
%
% Created:       01/09/23
% Last modified: 08/04/24
%--------------------------------------------------------------------------
% Copyright (c) 2024, Zhidong Lu. All rights reserved.
%
%--Modifications
%  08/04/24 a single solver for problem with/without path constraints
%  10/04/24 some codes are only evaluated in the first iteration
%  11/04/24 specify linprog options in the first iteration
%  18/04/24 add min/max option. switch to trust region update rule by doi
%  10.1137/030602563 Eq (3.4)

%% check problem and extract solver parameters
problem   = problem_in;

iter      = problem.solver.iter;             % current iteration
if iter >= problem.solver.iter_max
    problem.solved = 0;                      % maximum iterations reached
    return
end
if iter == 1
    check_problem(problem_in);               % check problem structure
end
iter_key  = problem.solver.iter_key;         % TR shrinking start iteration
gamma     = problem.solver.gamma;            % penalty weight of linearized constraints
gamma_tr  = problem.solver.gamma_tr;         % penalty weight of TRs
gamma_tr_s = problem.solver.gamma_tr_s;      % TR weight shrinking factor
delta_max = problem.solver.delta_max;        % current maximum TR size
delta_s   = problem.solver.delta_s;          % TR shrinking factor

problem.history.delta_max(iter) = delta_max; % record current max TR size
%% add a temp field in the problem for saving some intermediate vars
if ~isfield(problem,'tmp')
    problem.tmp = [];
else
    % extract temp vars
    no_path_cons = problem.tmp.no_path_cons;
    scale_e = problem.tmp.scale_e;
    if ~no_path_cons
        scale_Y = problem.tmp.scale_Y;
    end
    scaling_Z = problem.tmp.scaling_Z;
    S_Z_inv = problem.tmp.S_Z_inv;
    M = problem.tmp.M;
    lb0 = problem.tmp.lb0;
    ub0 = problem.tmp.ub0;
    solver_options = problem.tmp.solver_options;
end

%% force TR penalty weight to be small after a certain iteration
if iter > iter_key
    gamma_tr = max(1e-5, gamma_tr * gamma_tr_s);
    problem.solver.gamma_tr = gamma_tr;
    % accept ratio
    eta = 0;
else
    eta = -10; % accept poor iterations in the initial phase (exploration phase)
end
%% pack the optimziation variables
% the vector z = [dtf,dx1,dx2,...,dxN,du1,du2,...,duN]
[problem.zInit, problem.index.tf, problem.index.X, problem.index.U] = pack_opt_var(problem);
%% calculate output Y, defects E, and constraints C of the current solution
if ~isfield(problem.tmp,'no_path_cons')
    no_path_cons = problem.nOutput == 0 || all(isinf([problem.bounds.y_low;problem.bounds.y_upp]));
    problem.tmp.no_path_cons = no_path_cons;
end
if no_path_cons
    % collocation defects: e_hat = e - A*p = 0
    if iter <= 1
        [e, A] = build_E_jac(problem);
    else
        e = problem.solution.e;
        A = problem.solution.A;
    end
else
    % collocation defects: e_hat = e - A*p = 0
    % inequalitis: C_hat = C + dCdz*p  >= 0
    if iter <= 1
        [~, e, A, C, dCdz] = build_Y_E_C_jac(problem);
    else
        e = problem.solution.e;
        A = problem.solution.A;
        C = problem.solution.C;
        dCdz = problem.solution.dCdz;
    end
    % select non-inf elements in C and dCdz
    if ~isfield(problem.tmp, 'Y_bound')
        problem.tmp.Y_bound = [repmat(problem.bounds.y_low, problem.nGrid, 1);...
            repmat(problem.bounds.y_upp, problem.nGrid, 1)];
    end
    problem.index.Y_ninf = ~isinf(problem.tmp.Y_bound);
    c = C(problem.index.Y_ninf);
    c_z = dCdz(problem.index.Y_ninf, :);
end
%% define the scaling vectors
if ~isfield(problem.scale, 'z')
    problem.scale.z = ones(size(problem.zInit)); % pre-allocation
    problem.scale.z(problem.index.tf) = problem.scale.tf;
    problem.scale.z(problem.index.X) = repmat(problem.scale.x, 1, problem.nGrid);
    problem.scale.z(problem.index.U) = repmat(problem.scale.u, 1, problem.nGrid);
end

if ~isfield(problem.tmp, 'scale_e')
    scale_e = repmat(problem.scale.x, problem.nGrid-1, 1);
    problem.tmp.scale_e = scale_e;
end

if ~no_path_cons && ~isfield(problem.tmp, 'scale_Y')
    scale_Y = repmat(problem.scale.y, 2*problem.nGrid, 1);
    problem.tmp.scale_Y = scale_Y;
end
%% construct the subproblem, extended optimization variable
% Zall = [z; v_1,...,v_neq, w_1,...,w_neq, t_1,...,t_nieq, delta_1,...,delta_N+1];
N_X = problem.nState * problem.nGrid;
N_U = problem.nControl * problem.nGrid;
N_z = N_X + N_U + 1;
N_EQ = length(e);
if ~no_path_cons
    N_IE = length(c);
else
    N_IE = 0;             % no path IEQs
end
N_tr = problem.nGrid + 1; % tf and N-collocation nodes trust regions

N_Zall = N_z + 2*N_EQ + N_IE + N_tr;

% scaling for Z_all
if ~isfield(problem.tmp, 'S_Z_inv')
    scaling_EQ_v = scale_e;
    scaling_EQ_w = scale_e;
    if ~no_path_cons
        scaling_IEQ_t = scale_Y(problem.index.Y_ninf);
    else
        scaling_IEQ_t = [];    % no path IEQs
    end
    % scaling_tr = ones(N_tr, 1);
    scaling_Z = [problem.scale.z; scaling_EQ_v; scaling_EQ_w; scaling_IEQ_t; ones(N_tr, 1)];
    S_Z_inv = diag(1./scaling_Z);
    problem.tmp.S_Z_inv = S_Z_inv;
    problem.tmp.scaling_Z = scaling_Z;
end
%% construct the subproblem, cost function
% % real cost J
% f value:
% min tf problem: f = [1;0;...;0];
% min lag int problem: f = [0;0;...;1;...;0];
f0 = zeros(N_Zall, 1);
if problem.type == 0   % min tf
    tf0 = problem.zInit(problem.index.tf);
    if isfield(problem, 'min_max')
        if strcmp(problem.min_max,'max')
            f0(problem.index.tf) = -1;
        else
            f0(problem.index.tf) = 1;
        end
    else
        f0(problem.index.tf) = 1;
    end
else                   % min lag int
    Lint_f_Idx = problem.index.X(problem.nState, problem.nGrid);
    Lint_f = problem.zInit(Lint_f_Idx);
    if isfield(problem, 'min_max')
        if strcmp(problem.min_max,'max')
            f0(Lint_f_Idx) = -1;
        else
            f0(Lint_f_Idx) = 1;
        end
    else
        f0(Lint_f_Idx) = 1;
    end
end

% % penalty cost -- 1-norm of constr and trust region penalties
f1 = zeros(N_Zall, 1);
f1(N_z : (N_z + 2*N_EQ + N_IE)) = gamma;              % constraints penalty
f1((N_z + 2*N_EQ + N_IE+1):N_Zall) = gamma_tr;       % trust region penalty
f_vec_s = S_Z_inv*(f0 + f1);                           % total penalty cost

%% construct the subproblem, EQ/IEQ constraints
% % equality constraints, A_eq_s*Z_s = b_eq
A_eq_s = [A, eye(N_EQ), -eye(N_EQ), zeros(N_EQ, N_IE), zeros(N_EQ, N_tr)]*S_Z_inv;
b_eq_s = e;

% % A_ieq_s*Z_s <= b_ieq
if ~isfield(problem.tmp, 'M')
    % transition matrix M from delta_all (N_tr*1) to delta_vec (N_p*1)
    M = zeros(N_z, N_tr);
    M(1, 1) = 1;             % both the first elements are trust region for tf
    % assign each column
    for k = 1:problem.nGrid
        node_XU_idx = [problem.index.X(:, k); problem.index.U(:, k)];
        M(node_XU_idx, k+1) = 1;
    end
    problem.tmp.M = M;
end

if ~no_path_cons
    % three groups of inequalities
    % 1   -dCdz*dz - t <= C0
    % 2   -dz - M*delta_all <= 0
    % 3    dz - M*delta_all <= 0
    A_ieq = [-c_z, zeros(N_IE, N_EQ), zeros(N_IE, N_EQ), -eye(N_IE), zeros(N_IE, N_tr);
     -diag(problem.scale.z), zeros(N_z, N_EQ), zeros(N_z, N_EQ), zeros(N_z, N_IE), -M;
      diag(problem.scale.z), zeros(N_z, N_EQ), zeros(N_z, N_EQ), zeros(N_z, N_IE), -M];
    A_ieq_s = A_ieq*S_Z_inv;
    b_ieq_s = [c; zeros(2*N_z,1)];
else
    % TWO groups of inequalities
    % 1   -dz_s - M*delta_all <= 0
    % 2    dz_s - M*delta_all <= 0
    A_ieq = [-diag(problem.scale.z), zeros(N_z, N_EQ), zeros(N_z, N_EQ), -M;
              diag(problem.scale.z), zeros(N_z, N_EQ), zeros(N_z, N_EQ), -M];
    A_ieq_s = A_ieq*S_Z_inv;
    b_ieq_s = zeros(2*N_z,1);
end
%% construct the subproblem, box bounds
% % box bounds for z
if ~isfield(problem.tmp, 'lb0')
    % lb and ub for states and controls
    lb0 = -inf*ones(size(problem.zInit));
    ub0 =  inf*ones(size(problem.zInit));
    % tf
    lb0(problem.index.tf) = problem.bounds.Tf_low;
    ub0(problem.index.tf) = problem.bounds.Tf_upp;
    % state vector
    lb0(problem.index.X) = repmat(problem.bounds.x_low, 1, problem.nGrid);
    ub0(problem.index.X) = repmat(problem.bounds.x_upp, 1, problem.nGrid);
    % initial state
    lb0(problem.index.X(:,1)) = problem.bounds.xIBC_low;
    ub0(problem.index.X(:,1)) = problem.bounds.xIBC_upp;
    % final state
    lb0(problem.index.X(:,end)) = problem.bounds.xFBC_low;
    ub0(problem.index.X(:,end)) = problem.bounds.xFBC_upp;
    % control vector
    lb0(problem.index.U) = repmat(problem.bounds.u_low, 1, problem.nGrid);
    ub0(problem.index.U) = repmat(problem.bounds.u_upp, 1, problem.nGrid);
    
    if ~isempty(problem.bounds.u_IBC_low)
        % initial control
        lb0(problem.index.U(:,1)) = problem.bounds.u_IBC_low;
        ub0(problem.index.U(:,1)) = problem.bounds.u_IBC_upp;
    end
    if ~isempty(problem.bounds.u_FBC_low)
        % final control
        lb0(problem.index.U(:,end)) = problem.bounds.u_FBC_low;
        ub0(problem.index.U(:,end)) = problem.bounds.u_FBC_upp;
    end
    problem.tmp.lb0 = lb0;
    problem.tmp.ub0 = ub0;
end

% lb and ub for incrementals dz
lb = lb0 - problem.zInit;
ub = ub0 - problem.zInit;
% add box bounds for artifical slack variables and trust regions
lb_Z = [lb; zeros(2*N_EQ, 1); zeros(N_IE, 1); zeros(N_tr, 1)];
ub_Z = [ub; 1000*ones(2*N_EQ, 1); 1000*ones(N_IE, 1); ones(N_tr, 1)*delta_max];
% use 1000 instead of inf to avoid unboundness issue
% scaled lb and ub for Z_s
lb_Zs = lb_Z.*scaling_Z;
ub_Zs = ub_Z.*scaling_Z;

%% call the linear programming solver
% sparse matrices
A_ieq_s =  sparse(A_ieq_s);
b_ieq_s = sparse(b_ieq_s);
A_eq_s = sparse(A_eq_s);
b_eq_s = sparse(b_eq_s);

switch problem.solver.lp_solver
    case 'matlab_linprog'
        if ~isfield(problem.tmp,'solver_options')
            solver_options = optimoptions('linprog','Algorithm','interior-point',...
                'OptimalityTolerance',1e-6,'ConstraintTolerance',1e-5,'Display','off');
            problem.tmp.solver_options = solver_options;
        end
        [Zall_opt_s,fval,exitflag,output,lambda]  = linprog(f_vec_s,A_ieq_s,...
            b_ieq_s,A_eq_s, b_eq_s,lb_Zs,ub_Zs,solver_options);
    case 'gorubi_linprog'
        if ~isfield(problem.tmp,'solver_options')
            solver_options.Display = 'off';
            problem.tmp.solver_options = solver_options;
        end
        [Zall_opt_s,~,~,~]  = gurobi_linprog(f_vec_s,...
            A_ieq_s,b_ieq_s,A_eq_s, b_eq_s,lb_Zs,ub_Zs,solver_options);
    otherwise
        error('Error! Only matlab_linprog and gorubi_linprog are supported.')
end
z_opt_s = Zall_opt_s(1:N_z);   % extract physical optimization variables z
z_opt = z_opt_s./problem.scale.z;                       % unscale z_opt_s
step_size = norm(z_opt_s, inf);                      % calculate step size
problem.history.step_size(iter) = step_size;         % record step size
delta_all_new = Zall_opt_s((N_z + 2*N_EQ + N_IE + 1):N_Zall); % no scaling for TR
problem.history.delta_all(iter, :) = delta_all_new;
%% compute the merit BEFORE and AFTER the solution
% the merit function is the cost plus gamma*(1-norm of EQ/IEQ violations),
% trust region penalty term is not included.
% reshape e and C indices
e_idx = reshape(1:(problem.nGrid-1)*problem.nState, [], (problem.nGrid-1));
if ~no_path_cons
    C_idx = reshape(1:problem.nGrid*problem.nOutput, [], problem.nGrid);
    C_idx = [C_idx; C_idx + problem.nGrid*problem.nOutput];
end

% merit before the solution
if problem.type == 0
    J_old = tf0;
else
    J_old = Lint_f;
end
e_old_scaled = (problem.scale.x).*e(e_idx);
Jeq_old_vec = gamma * sum(abs(e_old_scaled), 1);
if ~no_path_cons
    C_old_scaled = [problem.scale.y;problem.scale.y].*C(C_idx);
    Jieq_old_vec = gamma * sum(max(0, -C_old_scaled), 1);
else
    Jieq_old_vec = 0;
end
if iter == 1
    delta_all_old = delta_max*ones(1, N_tr);
else
    delta_all_old = problem.history.delta_all(iter - 1, :);
end
Jtr_old = gamma_tr * sum(delta_all_old);
merit_old_sum = J_old + sum(Jeq_old_vec) + sum(Jieq_old_vec) + Jtr_old;

% predicted merit after the solution
if problem.type == 0
    J = z_opt(1) + tf0;
else
    J = z_opt(Lint_f_Idx) + Lint_f;
end
e_pre = e - A*z_opt;
e_pre_scaled = (problem.scale.x).*e_pre(e_idx);
Jeq_pre_vec = gamma * sum(abs(e_pre_scaled), 1);
if ~no_path_cons
    C_pre = C + dCdz*z_opt;
    C_pre_scaled = [problem.scale.y;problem.scale.y].*C_pre(C_idx);
    Jieq_pre_vec = gamma * sum(max(0, -C_pre_scaled), 1);
else
    Jieq_pre_vec = 0;
end
Jtr_pre = gamma_tr * sum(delta_all_new);
merit_pre_sum = J + sum(Jeq_pre_vec) + sum(Jieq_pre_vec) + Jtr_pre;

% real merit after the solution
if ~no_path_cons
    [y_new, e_new, A_new, C_new, dCdz_new] = build_Y_E_C_jac(problem, problem.zInit + z_opt);
    e_new_scaled = (problem.scale.x).*e_new(e_idx);
    Jeq_new_vec = gamma * sum(abs(e_new_scaled), 1);
    C_new_scaled = [problem.scale.y;problem.scale.y].*C_new(C_idx);
    Jieq_new_vec = gamma * sum(max(0, -C_new_scaled), 1);
    merit_new_sum = J + sum(Jeq_new_vec) + sum(Jieq_new_vec) + Jtr_pre;
else
    [e_new, A_new] = build_E_jac(problem, problem.zInit + z_opt);
    e_new_scaled = (problem.scale.x).*e_new(e_idx);
    Jeq_new_vec = gamma * sum(abs(e_new_scaled), 1);
    merit_new_sum = J + sum(Jeq_new_vec) + Jtr_pre;
end
rho = (merit_old_sum - merit_new_sum)/(max(1e-4, merit_old_sum - merit_pre_sum));
problem.history.rho(iter) = rho;
%% update maximum trust region for next iteration
% if rho < 1/4
%     delta_max_next = 1/4*delta_max;
%     if iter>1 && problem.history.rho(iter-1) < 0.25
%         delta_max_next = 1/10*delta_max; % reduce the TR size further
%         problem.solver.gamma = min(1e3, 2*problem.solver.gamma); % increase penalty weight
%     end
%     %     if iter>2 && max(problem.history.rho([iter-1;iter])) < 0.25
%     %         delta_max_next = 1/30*delta_max; % reduce the TR size further
%     %         problem.solver.gamma = min(1e3, 5*problem.solver.gamma); % increase penalty weight
%     %     end
% elseif rho > 0.91 && abs(step_size - delta_max) < 1e-4
%     delta_max_next = 2*delta_max;
% else
%     delta_max_next = delta_max;
% end

%% update rule proposed in ''A Note on Trust-Region Radius Update'' 2005
if rho < 0
    ratio = max(0.1, 0.5 + 0.05*rho);
elseif rho < 0.95 
    ratio = 0.5 + (1 - 0.5)*(rho/0.95)^2;
elseif abs(step_size - delta_max) < 1e-4
    ratio = 1.01 + (2 - 1.01)*exp(-((rho - 1)/(0.95 - 1))^2);
else
    ratio = 1;
end
delta_max_next = ratio * delta_max;

%% artificial maximum TR size bound (decreasing with iters)
delta_max_init = problem.history.delta_max(1);
iter_excess = max(0, iter - iter_key);
delta_lim = delta_max_init*(delta_s^iter_excess);
delta_max_next = min(delta_max_next, delta_lim); % enforce decreasing limit
problem.solver.delta_max = delta_max_next; % record TR size for the next iter
%% update solution to the problem
if rho > eta
    iter_success = 1;                     % current iteration is successful
    % update parameter (tf)
    problem.tf = z_opt(problem.index.tf) + problem.zInit(problem.index.tf);
    % update states and controls
    problem.states = z_opt(problem.index.X) + problem.zInit(problem.index.X);
    problem.controls = z_opt(problem.index.U) + problem.zInit(problem.index.U);
    
    % record solution values based on the updated variables
    % updated cost and merit
    problem.history.J(iter) = J;
    problem.history.merit(iter) = merit_new_sum;
    % record matrices for the next iteration
    problem.solution.A = A_new;
    problem.solution.e = e_new;
    if ~no_path_cons
        problem.solution.C = C_new;
        problem.solution.dCdz = dCdz_new;
        % update ouput
        problem.outputs = y_new;
    end
else
    iter_success = 0;
    % no update on tf, states, or controls
    % keep old costs
    problem.history.J(iter) = J_old;
    problem.history.merit(iter) = merit_old_sum;
    % keep matrices for the next iter
    problem.solution.A = A;
    problem.solution.e = e;
    if ~no_path_cons
        problem.solution.C = C;
        problem.solution.dCdz = dCdz;
    end
end

%% calculate optimality and feasibility
opti_relative =  abs(problem.history.J(iter)- J_old)/J_old; % local optimality (relative)
EQ_vio = norm((scale_e).*(problem.solution.e), inf);           % EQ violation
problem.history.opti_relative(iter) = opti_relative;
problem.history.EQ_vio(iter) = EQ_vio;
if ~no_path_cons
    IE_vio = norm(max(0, -scale_Y.*(problem.solution.C)), inf);  % IEQ violation
    problem.history.IE_vio(iter) = IE_vio;
    cons_vio = max(EQ_vio,IE_vio);
else
    cons_vio = EQ_vio;
end
%% check stopping criteria
if cons_vio <= problem.solver.constr_tol
    if iter_success && opti_relative <= problem.solver.opt_tol
        problem.solved = 1; % local optimality within tolerance and constraint violation within tolerance
    elseif step_size <= problem.solver.step_tol
        problem.solved = 2; % dz within step tolerance and constraint violation within tolerance
    end
else
    if step_size <= problem.solver.step_tol
        problem.solved = 3; % dz within step tolerance and constraints are still violated
    else
        problem.solved = -1; % no feasible solution found yet, need more iteration
    end
end
end