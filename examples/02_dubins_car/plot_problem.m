figure(1);

N = problem.nState;

subplot(N,1,1)
hold on
plot(problem.tau_vec*problem.tf,problem.states(1,:),'linewidth',1)
ylabel('x')

subplot(N,1,2)
hold on
plot(problem.tau_vec*problem.tf,problem.states(2,:),'linewidth',1)
ylabel('y')


subplot(N,1,3)
hold on
plot(problem.tau_vec*problem.tf,problem.states(3,:),'linewidth',1)
ylabel('\theta')

if N>3
    subplot(N,1,4)
    hold on
    plot(problem.tau_vec*problem.tf,problem.states(4,:),'linewidth',1)
    ylabel('Q')
end
%%
figure(2);
subplot(2,1,1)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(1,:),'linewidth',1)
ylabel('V')

subplot(2,1,2)
hold on
plot(problem.tau_vec*problem.tf,problem.controls(2,:),'linewidth',1)
ylabel('\omega')
