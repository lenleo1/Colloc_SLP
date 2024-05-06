figure(1);

subplot(4,2,1)
hold on
plot(problem.tau_vec*problem.tf,problem.states(1,:),'linewidth',1)
ylabel('px')

subplot(4,2,2)
hold on
plot(problem.tau_vec*problem.tf,problem.states(2,:),'linewidth',1)
ylabel('py')

subplot(4,2,3)
hold on
plot(problem.tau_vec*problem.tf,problem.states(3,:),'linewidth',1)
ylabel('vx')

subplot(4,2,4)
hold on
plot(problem.tau_vec*problem.tf,problem.states(4,:),'linewidth',1)
ylabel('vy')

subplot(4,2,5)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(1,:),'linewidth',1)
ylabel('ux')

subplot(4,2,6)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(2,:),'linewidth',1)
ylabel('uy')

subplot(4,2,7.5)
hold on
plot(problem.tau_vec*problem.tf,problem.states(5,:),'linewidth',1)
ylabel('Q')