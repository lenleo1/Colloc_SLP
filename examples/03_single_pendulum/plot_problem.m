figure(1);

subplot(3,1,1)
hold on
plot(problem.tau_vec*problem.tf,problem.states(1,:),'linewidth',1)
ylabel('\theta')
title('Single Pendulum Swing-Up');

subplot(3,1,2)
hold on
plot(problem.tau_vec*problem.tf,problem.states(2,:),'linewidth',1)
ylabel('q')

subplot(3,1,3)
hold on
plot(problem.tau_vec*problem.tf,problem.controls,'linewidth',1)
ylabel('u')

% if size(problem.states,1)>=3
%     subplot(3,1,4)
%     hold on
%     plot(problem.tau_vec*problem.tf,problem.states(3,:),'linewidth',1)
%     ylabel('integartion')
% end

