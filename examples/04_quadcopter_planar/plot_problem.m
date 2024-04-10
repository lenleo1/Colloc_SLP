figure(1);

subplot(2,4,1)
hold on
plot(problem.tau_vec*problem.tf,problem.states(1,:),'linewidth',1)
ylabel('x')

subplot(2,4,2)
hold on
plot(problem.tau_vec*problem.tf,problem.states(2,:),'linewidth',1)
ylabel('y')

subplot(2,4,3)
hold on
plot(problem.tau_vec*problem.tf,57.3*problem.states(3,:),'linewidth',1)
ylabel('\theta')

subplot(2,4,4)
hold on
plot(problem.tau_vec*problem.tf,problem.states(4,:),'linewidth',1)
ylabel('dx')


subplot(2,4,5)
hold on
plot(problem.tau_vec*problem.tf,problem.states(5,:),'linewidth',1)
ylabel('dy')

subplot(2,4,6)
hold on
plot(problem.tau_vec*problem.tf,problem.states(6,:),'linewidth',1)
ylabel('q')

subplot(2,4,7)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(1,:),'linewidth',1)
ylabel('f1')

subplot(2,4,8)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(2,:),'linewidth',1)
ylabel('f2')
