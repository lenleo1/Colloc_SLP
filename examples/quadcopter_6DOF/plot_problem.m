figure(1);
subplot(4,4,1)
hold on
plot(problem.tau_vec*problem.tf,problem.states(1,:),'linewidth',1)
ylabel('$X~(m)$', 'interpreter', 'latex')

subplot(4,4,2)
hold on
plot(problem.tau_vec*problem.tf,problem.states(2,:),'linewidth',1)
ylabel('$Y~(m)$', 'interpreter', 'latex')

subplot(4,4,3)
hold on
plot(problem.tau_vec*problem.tf,problem.states(3,:),'linewidth',1)
ylabel('$Z~(m)$', 'interpreter', 'latex')

subplot(4,4,4)
hold on
plot(problem.tau_vec*problem.tf,57.3*problem.states(4,:),'linewidth',1)
ylabel('$\theta~(^\circ)$', 'interpreter', 'latex')

subplot(4,4,5)
hold on
plot(problem.tau_vec*problem.tf,57.3*problem.states(5,:),'linewidth',1)
ylabel('$\phi~(^\circ)$', 'interpreter', 'latex')

subplot(4,4,6)
hold on
plot(problem.tau_vec*problem.tf,57.3*problem.states(6,:),'linewidth',1)
ylabel('$\psi~(^\circ)$', 'interpreter', 'latex')

subplot(4,4,7)
hold on
plot(problem.tau_vec*problem.tf,problem.states(7,:),'linewidth',1)
ylabel('$u_E~(m/s)$', 'interpreter', 'latex')

subplot(4,4,8)
hold on
plot(problem.tau_vec*problem.tf,problem.states(8,:),'linewidth',1)
ylabel('$v_E~(m/s)$', 'interpreter', 'latex')

subplot(4,4,9)
hold on
plot(problem.tau_vec*problem.tf,problem.states(9,:),'linewidth',1)
ylabel('$w_E~(m/s)$', 'interpreter', 'latex')

subplot(4,4,10)
hold on
plot(problem.tau_vec*problem.tf, 57.3*problem.states(10,:),'linewidth',1)
ylabel('$\dot{\theta}~(^\circ/s)$', 'interpreter', 'latex')

subplot(4,4,11)
hold on
plot(problem.tau_vec*problem.tf, 57.3*problem.states(11,:),'linewidth',1)
ylabel('$\dot{\phi}~(^\circ/s)$', 'interpreter', 'latex')

subplot(4,4,12)
hold on
plot(problem.tau_vec*problem.tf, 57.3*problem.states(12,:),'linewidth',1)
ylabel('$\dot{\psi}~(^\circ/s)$', 'interpreter', 'latex')
%%
subplot(4,4,13)
hold on
plot(problem.tau_vec*problem.tf, problem.controls  (1,:),'linewidth',1)
ylabel('$u_1~(-)$', 'interpreter', 'latex')


subplot(4,4,14)
hold on
plot(problem.tau_vec*problem.tf, problem.controls  (2,:),'linewidth',1)
ylabel('$u_2~(-)$', 'interpreter', 'latex')

subplot(4,4,15)
hold on
plot(problem.tau_vec*problem.tf, problem.controls  (3,:),'linewidth',1)
ylabel('$u_3~(-)$', 'interpreter', 'latex')

subplot(4,4,16)
hold on
plot(problem.tau_vec*problem.tf, problem.controls  (4,:),'linewidth',1)
ylabel('$u_4~(-)$', 'interpreter', 'latex')