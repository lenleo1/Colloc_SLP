function y_smooth = smooth_interp(y, K)
% smoothInterpolation interpolates a given data vector y using cubic splines.
% Inputs:
%   y  - The original data vector (1xN or Nx1)
%   K  - Refinement factor: the original time interval is divided by K
% Written by:    Zhidong Lu
% e-mail:        zhidong.lu@tum.de
%
% Created:       01/09/23
% Last modified: 18/04/24
%--------------------------------------------------------------------------
% Copyright (c) 2024, Zhidong Lu. All rights reserved.

% Ensure the data vector is a row vector
if iscolumn(y)
    y = y';
end

% Number of data points
N = length(y);

% Original time nodes
dt = 1;
t = linspace(0, dt * (N - 1), N);

% Refined time nodes for interpolation
t_fine = linspace(0, dt * (N - 1), K * (N - 1) + 1);

% Perform cubic spline interpolation
y_smooth = interp1(t, y, t_fine, 'spline');

%     % Plotting for visual verification (optional)
%     figure;
%     plot(t, y, 'o', 'MarkerFaceColor', 'r');  % Original data points
%     hold on;
%     plot(t_fine, y_smooth, 'b-');  % Interpolated curve
%     title('Smooth Interpolation of Data using Cubic Spline');
%     xlabel('Time');
%     ylabel('Data Value');
%     legend('Original Data', 'Interpolated Data', 'Location', 'Best');
%     grid on;
end
