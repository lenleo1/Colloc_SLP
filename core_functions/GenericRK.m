function [x, y] = GenericRK(method, modelhandle, t_real, x0, u, N_out)
% Generic Runge Kutta integrator for explicit methods =====================
% This integrator uses the general definition of Runge Kutta (RK) Methods
% together with their respective Butcher tableaus to enable the modular use of
% different RK-methods in one function file (by utilizing the Butcher tableau 
% corresponding to the integration method specified by the user).
% 
% Description of Inputs:
%   NAME        SIZE                 TYPE                DESCRIPTION        
%   method      -                    String              String defining the integration method (e.g. 'ForwardEuler')
%   modelhandle -                    MATLAB-Function     Model-Function [xdot,y] = modelhandle(x,u)
%   t_real      [1          x N_TS]  Double              Time grid used for integration [t1, t2,...tN]
%   x0          [N_states   x    1]  Double              Statevector of initial Position (t=t0)
%   u           [N_controls x N_TS]  Double              Statevector for the whole integration interval
%   N_out       [1          x    1]  Double              Number of model outputs (NOUT)
%
% Description of Outputs:
%   NAME        SIZE                 TYPE                DESCRIPTION        
%   x           [N_states   x N_TS]  Double              State  vector for the time grid "t_real"  
%   y           [N_out      x N_TS]  Double              Output vector for the time grid "t_real"


%% Butcher Array Selection
%Assign the respective Butcher tableaus to matrices a,b depending on the user-specified "method". 
switch method
    case 'ForwardEuler'
        a = 0;
        b = 1;
    case 'HeunMethod'
        a = [0  , 0;...
             1  , 0];
        b = [1/2, 1/2];
    case '3rdOrder'
        a = [0 , 0, 0;...
            1/2, 0, 0;...
            -1 , 2, 0];
        b = [1/6, 4/6, 1/6];
    case '4thOrder'
        a = [0 ,0  ,0 ,0;...
            1/2,0  ,0 ,0;...
            0  ,1/2,0 ,0;...
            0  ,0  ,1 ,0];
        b = [1/6, 1/3, 1/3, 1/6];
end

%% Initialize Variables
% Save the number of elements in the Runge Kutta Butcher array b. 
rkorder = length(b);

% Calculate an array containing the step size for each integration step
% from the time grid vector "t_real".
ht = diff(t_real);

% Calculate the dimensions for the number of states and time
% steps and save them in variables.
N_states    = size(x0,1);
N_TS        = length(t_real);

% Initialize the matrices ("x" (states) and "y" (outputs), returned 
% by the RK-integrator) with zeros.
x  = zeros(N_states, N_TS);
y  = zeros(N_out, N_TS);

% Initialize two variables ("Fx", "Fy") to store the Runge Kutta
% evaluations within one integration step.  
Fx = zeros(N_states, rkorder);
Fy = zeros(N_out, rkorder);
%% Runge Kutta Integration Loop (Time Steps)

% Write the initial state "x0" in the respective column of the state
% matrix "x".
x(:,1) = x0;

% Use a for-block to go through all integration time steps.
for i = 2:N_TS
    % Calculate the current time "ind_timestep" step, the current 
    % step size "ht_ind", the current state "x_ind" and the current
    % control "u_ind"
    ind_timestep = i-1;
    ht_ind       = ht(ind_timestep);
    x_ind        = x(:,ind_timestep);
    u_ind        = u(:,ind_timestep);    
    % Runge Kutta Internal Loop (Within One Time Step)  
    % Calculate the internal function evaluations ("Fx", "Fy") using
    %       the butcher arrays defined above. Use the temporary variable
    %       "x_tmp" and the current control u_ind to call the model-function:
    %       modelhandle(x_tmp, u_ind).
    for jj = 1:rkorder
        x_tmp = x_ind;
        % calculate xtmp
        for ll = 1:jj-1
            x_tmp = x_tmp + ht_ind * a(jj,ll) * Fx(:,ll);
        end
        % Evaluate Model
        if N_out
            [Fx(:,jj), Fy(:,jj)] = modelhandle(x_tmp, u_ind);
        else
            Fx(:,jj) = modelhandle(x_tmp, u_ind);
        end
    end
    % Use the computed internal function evaluations to calculate the
    %       state derivatives "xdot" which will later-on be used to perform
    %       the integration step. Also, calculate the current output "y"
    xdot = zeros(size(x0));
    for jj = 1:rkorder
        xdot = xdot + b(jj) * Fx(:,jj);
        if N_out
            y(:, ind_timestep)  = y(:, ind_timestep) + b(jj) * Fy(:,jj);
        end
    end
    % Perform the integration step to obtain the next state.
    x(:,i) = x_ind + ht_ind * xdot;
end

% If the model has outputs get the output for the last time step by
%       one additional model evaluation.
if N_out
    [~, y(:,N_TS)] = modelhandle(x(:,N_TS), u(:,N_TS));
else
    y = [];
end
end