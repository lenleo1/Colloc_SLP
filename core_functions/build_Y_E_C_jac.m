function [y_rec, e, A, C, dCdz] = build_Y_E_C_jac(problem,varargin)
% 
% Build outputs vector, constraints vector and their jacobians wrt z
% z=[dt_f,dx_1,dx_2,…,dx_N,du_1,du_2,…,du_N ]^T
%
% Version 1.0
% usage: [y_rec, e, A, C, dCdz] = build_Y_E_C_jac(problem,varargin)
% input: problem       - the problem to be solved
%        varargin      - (optional) specify the z vector, if not, extract z from problem    
% output: y_rec        - the outputs vector                (nOutput, nGrid)
%         e            - collocation defects vector        ( ? * 1)
%         A            - de/dz                             ( ? * nz)
%         C            - inequality constraints            (2*nY * 1)
%         dCdz         - jacobian of C wrt z               ( ? * nz)
%
% the structure of C >= 0 includes lower bounds and upper bounds:
%  Y - Y_lb >= 0
% -Y + Y_ub >= 0
% Written by:    Zhidong Lu
% e-mail:        zhidong.lu@tum.de
%
% Created:       01/09/23
% Last modified: 08/04/24
%--------------------------------------------------------------------------
% Copyright (c) 2024, Zhidong Lu. All rights reserved.
%
%--Modifications

% two options, the first is to provide the exact solution
% the second is to extract from the problem struct
if nargin > 1 
    zInit = varargin{1};
    tf = zInit(problem.index.tf);
    x = reshape(zInit(problem.index.X),[],problem.nGrid);
    u = reshape(zInit(problem.index.U),[],problem.nGrid);
else
    tf = problem.tf;
    x =  problem.states;
    u =  problem.controls;
end

d_tau = problem.tau_vec(2) - problem.tau_vec(1);
N_X = problem.nState * problem.nGrid;
N_U = problem.nControl * problem.nGrid;
N_z = N_X + N_U + 1;
N_Y = problem.nOutput * problem.nGrid;

% preallocate
e = zeros(N_X - problem.nState, 1);
A  = zeros(N_X - problem.nState, N_z);
C  = zeros(2*N_Y, 1);
dCdz = zeros(2*N_Y, N_z);
%% evaluate the model function at each node
% preallocate
f_rec = nan(problem.nState, problem.nGrid);
dfdx_rec = nan(problem.nState, problem.nState, problem.nGrid);
dfdu_rec = nan(problem.nState, problem.nControl, problem.nGrid);
y_rec = nan(problem.nOutput, problem.nGrid);
dydx_rec = nan(problem.nOutput, problem.nState, problem.nGrid);
dydu_rec = nan(problem.nOutput, problem.nControl, problem.nGrid);

sys = problem.sys;
for k = 1 : problem.nGrid
    [f_rec(:,k), dfdx_rec(:,:,k), dfdu_rec(:,:,k),...
        y_rec(:,k), dydx_rec(:,:,k), dydu_rec(:,:,k)] = sys(x(:,k),u(:,k));
end

% scale to the [0,1] time grid
g_rec = f_rec * tf;
dgdx_rec = dfdx_rec * tf;
dgdu_rec = dfdu_rec * tf;
%% compute e, A, C, and dCdz
for k = 1 : problem.nGrid
    if k < problem.nGrid
        % compute collocation defect
        CD = x(:,k+1) - (x(:,k) + 0.5*(g_rec(:,k) + g_rec(:,k+1))*d_tau);
        % compute Jacobian matrices F
        FXs = 0.5*d_tau*dgdx_rec(:,:,k)   + eye(problem.nState);
        FXe = 0.5*d_tau*dgdx_rec(:,:,k+1) - eye(problem.nState);
        FUs = 0.5*d_tau*dgdu_rec(:,:,k);
        FUe = 0.5*d_tau*dgdu_rec(:,:,k+1);
        Ftf = 0.5*d_tau*(f_rec(:,k) + f_rec(:,k+1));
        % assemble E
        row_index1 = problem.nState*(k-1) + 1 : problem.nState*(k);
        e(row_index1)  =  CD;
        % assemble A
        A(row_index1, problem.index.tf) = Ftf;
        A(row_index1, problem.index.X(:,k)) = FXs;
        A(row_index1, problem.index.X(:,k+1)) = FXe;
        A(row_index1, problem.index.U(:,k)) = FUs;
        A(row_index1, problem.index.U(:,k+1)) = FUe;
    end
    
    % assemble C
    y_lb = problem.bounds.y_low;
    y_ub = problem.bounds.y_upp;
    
    row_idx_l = problem.nOutput*(k-1) + 1 : problem.nOutput*k;
    C(row_idx_l) = y_rec(:,k) - y_lb;
    dCdz(row_idx_l, problem.index.X(:,k)) = dydx_rec(:,:,k);
    dCdz(row_idx_l, problem.index.U(:,k)) = dydu_rec(:,:,k);
    
    row_idx_u = N_Y + problem.nOutput*(k-1) + 1 : (N_Y  + problem.nOutput*k);
    C(row_idx_u) = -y_rec(:,k) + y_ub;
    dCdz(row_idx_u, problem.index.X(:,k)) = -dydx_rec(:,:,k);
    dCdz(row_idx_u, problem.index.U(:,k)) = -dydu_rec(:,:,k);
end

end
