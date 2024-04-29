figure(1);

subplot(3,1,1)
hold on
plot(problem.tau_vec*problem.tf,problem.states(1,:),'linewidth',1)
ylabel('x')
title('Double integrator minimum time');

subplot(3,1,2)
hold on
plot(problem.tau_vec*problem.tf,problem.states(2,:),'linewidth',1)
ylabel('v')

subplot(3,1,3)
hold on
plot(problem.tau_vec*problem.tf,problem.controls,'linewidth',1)
ylabel('u')