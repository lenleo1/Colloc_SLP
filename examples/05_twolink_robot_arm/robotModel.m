function xdot=robotModel(x,u)
omega_alpha = x(1);
omega_beta  = x(2);
theta = x(3);
u1 = u(1);
u2 = u(2);

omega_alpha_dot = (sin(theta).*(9/4*cos(theta).*omega_alpha.^2)+2*omega_beta.^2 + 4/3*(u1-u2) - 3/2*cos(theta).*u2 )./ (31/36 + 9/4*sin(theta).^2);
omega_beta_dot = -( sin(theta).*(9/4*cos(theta).*omega_beta.^2)+7/2*omega_alpha.^2 - 7/3*u2 + 3/2*cos(theta).*(u1-u2) )./ (31/36 + 9/4*sin(theta).^2);
theta_dot = omega_beta-omega_alpha;
alpha_dot = omega_alpha;
xdot = [omega_alpha_dot; omega_beta_dot; theta_dot; alpha_dot];