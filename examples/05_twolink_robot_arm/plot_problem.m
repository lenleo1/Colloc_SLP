figure(1);

subplot(3,2,1)
hold on
plot(problem.tau_vec*problem.tf,problem.states(1,:),'linewidth',1)
ylabel('\omega_{\alpha}')

subplot(3,2,2)
hold on
plot(problem.tau_vec*problem.tf,problem.states(2,:),'linewidth',1)
ylabel('\omega_{\beta}')

subplot(3,2,3)
hold on
plot(problem.tau_vec*problem.tf,problem.states(3,:),'linewidth',1)
ylabel('\theta')

subplot(3,2,4)
hold on
plot(problem.tau_vec*problem.tf,problem.states(4,:),'linewidth',1)
ylabel('\alpha')

subplot(3,2,5)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(1,:),'linewidth',1)
ylabel('u1')

subplot(3,2,6)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(2,:),'linewidth',1)
ylabel('u2')