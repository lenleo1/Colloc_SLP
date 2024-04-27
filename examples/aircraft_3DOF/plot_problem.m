%%
figure(1);

subplot(2,5,1)
hold on
plot(problem.tau_vec*problem.tf,problem.states(1,:),'linewidth',1)
ylabel('$X~(m)$', 'interpreter', 'latex')

subplot(2,5,2)
hold on
plot(problem.tau_vec*problem.tf,problem.states(2,:),'linewidth',1)
ylabel('$Y~(m)$', 'interpreter', 'latex')

subplot(2,5,3)
hold on
plot(problem.tau_vec*problem.tf,problem.states(3,:),'linewidth',1)
ylabel('$Z~(m)$', 'interpreter', 'latex')

subplot(2,5,4)
hold on
plot(problem.tau_vec*problem.tf,problem.states(4,:),'linewidth',1)
ylabel('$V~(m/s)$', 'interpreter', 'latex')

subplot(2,5,5)
hold on
plot(problem.tau_vec*problem.tf,57.3*problem.states(5,:),'linewidth',1)
ylabel('$\chi~^\circ$', 'interpreter', 'latex')

subplot(2,5,6)
hold on
plot(problem.tau_vec*problem.tf,57.3*problem.states(6,:),'linewidth',1)
ylabel('$\gamma~^\circ$', 'interpreter', 'latex')

subplot(2,5,7)
hold on
plot(problem.tau_vec*problem.tf,57.3*problem.controls(1,:),'linewidth',1)
ylabel('$\alpha~^\circ$', 'interpreter', 'latex')

subplot(2,5,8)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(2,:),'linewidth',1)
ylabel('$\delta_T$', 'interpreter', 'latex')

subplot(2,5,9)
hold on
plot(problem.tau_vec*problem.tf,57.3*problem.controls(3,:),'linewidth',1)
ylabel('$\mu~^\circ$', 'interpreter', 'latex')

% compute the output vector
for k = 1:problem.nGrid
    [~,~,~,problem.outputs(:, k),~,~] = problem.sys(problem.states(:,k), problem.controls(:,k));
end

subplot(2,5,10)
hold on
plot(problem.tau_vec*problem.tf, problem.outputs,'linewidth',1)
ylabel('$n_z$', 'interpreter', 'latex')


