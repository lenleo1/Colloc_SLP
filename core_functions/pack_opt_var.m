function [z,tfIdx,XIdx,UIdx] = pack_opt_var(problem)
% 
% Pack Optimization Variables into a Vector
% This function collapses state (x) and control (u) matricies into a single vector
% z=[dt_f,dx_1,dx_2,…,dx_N,du_1,du_2,…,du_N ]^T
%
% Version 1.0
% usage: [z,tfIdx,XIdx,UIdx] = pack_opt_var(problem)
% input: problem       - the problem to be solved
% output: z            - the optimization variables vector (N_z * 1)
%         tfIdx        - time duration index in z          ( 1 * 1)
%         XIdx         - all states indices in z           (nState * nGrid)
%         UIdx         - all controls indices in z         (nControl * nGrid)
%
% Written by:    Zhidong Lu
% e-mail:        zhidong.lu@tum.de
%
% Created:       01/09/23
% Last modified: 08/04/24
%--------------------------------------------------------------------------
% Copyright (c) 2024, Zhidong Lu. All rights reserved.
%
%--Modifications


nGrid = problem.nGrid;
nState = problem.nState;
nControl = problem.nControl;
X = problem.states;
U = problem.controls;
tf = problem.tf;


XCol = reshape(X, nState*nGrid, 1);
UCol = reshape(U, nControl*nGrid, 1);

% index of state, control variables in the z vector
tfIdx = 1;
XIdx = reshape(2:(numel(X)+1), nState, []);
UIdx = reshape((numel(X)+2):(numel(X)+1+numel(U)), nControl, []);

% opt variables are indexed so that the defects gradients appear as a banded matrix
z = zeros(numel(X)+numel(U)+1, 1); 
z(tfIdx)   = tf;  % d_tf is the first element
z(XIdx(:)) = XCol;
z(UIdx(:)) = UCol;
end