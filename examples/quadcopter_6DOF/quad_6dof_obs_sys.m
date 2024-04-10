function [xdot,Jx,Ju,y,Jyx,Jyu] = quad_6dof_obs_sys(states,controls)
% quad_6dof_sys
%% call the mex function
% statedot
[xdot, jac_xdot] = quad_sys_aug_Falcon(states, controls);
% output, y= 'obs_cons'
[y, jac_y] = fm_mex_ObstacleConstraint(states, controls);
%% statedot jacobian
Jx = jac_xdot(:, 1:13);
Ju = jac_xdot(:, 14:17); % u1 u2 u3 u4
%% output jacobian
Jyx = jac_y(1:13);
Jyu = jac_y(14:17); % u1 u2 u3 u4
end