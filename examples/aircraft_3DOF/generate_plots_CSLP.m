clear;clc
%% load data, falcon
load('CSLP_solution_240424.mat');
%% smooth states
tf = problem.tf;
states = problem.states;
controls = problem.controls;
outputs = problem.outputs;

time = linspace(0,tf,31);
for k = 1:6
    states_interp(k,:) = smooth_interp(states(k,:), 3);
end
time_interp = linspace(0, tf, length(states_interp));
%%
figure(1);
hold on
grid on
plot3(states_interp(1, :), states_interp(2, :), 1000-states_interp(3, :),...
    'linewidth',1);
set(gca,'YDir','reverse');
pbaspect([5,2,1])
xlabel('$X~(m)$', 'interpreter', 'latex')
ylabel('$Y~(m)$', 'interpreter', 'latex')
zlabel('$H~(m)$', 'interpreter', 'latex')
%%
figure(2);
subplot(5,2,1)
hold on; grid on
plot(time_interp, states_interp(1,:),'linewidth',1)
ylabel('$X~(m)$', 'interpreter', 'latex')

subplot(5,2,2)
hold on; grid on
plot(time_interp, states_interp(2,:),'linewidth',1)
ylabel('$Y~(m)$', 'interpreter', 'latex')

subplot(5,2,3)
hold on; grid on
plot(time_interp, 1000-states_interp(3,:),'linewidth',1)
ylabel('$H~(m)$', 'interpreter', 'latex')

subplot(5,2,4)
hold on; grid on
plot(time_interp, states_interp(4,:),'linewidth',1)
ylabel('$V~(m/s)$', 'interpreter', 'latex')

subplot(5,2,5)
hold on; grid on
plot(time_interp, 57.3*states_interp(5,:),'linewidth',1)
ylabel('$\chi~(^\circ)$', 'interpreter', 'latex')

subplot(5,2,6)
hold on; grid on
plot(time_interp, 57.3*states_interp(6,:),'linewidth',1)
ylabel('$\gamma~(^\circ)$', 'interpreter', 'latex')

subplot(5,2,7)
hold on; grid on
plot(time, 57.3*controls(1,:),'linewidth',1)
ylabel('$\alpha~(^\circ)$', 'interpreter', 'latex')

subplot(5,2,8)
hold on; grid on
plot(time, controls(2,:),'linewidth',1)
ylabel('$\delta_T~(-)$', 'interpreter', 'latex')

subplot(5,2,9)
hold on
plot(time, 57.3*controls(3,:),'linewidth',1)
ylabel('$\mu~(^\circ)$', 'interpreter', 'latex')

subplot(5,2,10)
hold on; grid on
plot(time, outputs(1,:),'linewidth',1)
ylabel('$n_z~(-)$', 'interpreter', 'latex')
%%
xlabel('$t~(s)$', 'interpreter', 'latex')
%% iteration process SLP iteration
figure(3);
subplot(1,2,1)
hold on
plot(0:1:6,[53.8;problem.history.J(1:6)],'linewidth',1)
xlabel('Iteration number', 'interpreter', 'latex');
ylabel('Cost value $t_f$', 'interpreter', 'latex');


subplot(1,2,2)
hold on
plot(0:1:6,[0.0667;problem.history.EQ_vio(1:6)],'linewidth',1)
xlabel('Iteration number', 'interpreter', 'latex');
ylabel('$\|\mathbf{e}\|_{\infty}$', 'interpreter', 'latex');
set(gca, 'YScale', 'log')
%% J and merit, rho and delta_max, delta_all
figure(4)
subplot(3,1,1);
hold on
plot(0:1:6, [53.8;problem.history.J(1:6)],'linewidth',1)
plot(0:1:6, [345.833;problem.history.merit(1:6)],'linewidth',1)
xlabel('Iteration number', 'interpreter', 'latex');
ylabel('Cost values', 'interpreter', 'latex');
legend({'$J$','$J_{\rm total}$'}, 'interpreter', 'latex');

subplot(3,1,2);
hold on
plot(0:1:6, [nan;problem.history.rho(1:6)],'linewidth',1)
plot(0:1:6, [5;problem.history.delta_max(1:6)],'linewidth',1)
xlabel('Iteration number', 'interpreter', 'latex');
legend({'$\rho$','$\Delta_{\rm max}$'}, 'interpreter', 'latex');

subplot(3,1,3);
hold on
plot(1:1:32, [5*ones(1,32);problem.history.delta_all],'linewidth',1)
set(gca, 'YScale', 'log')
xlabel('Collocation node index', 'interpreter', 'latex');
ylabel('Trust region $\mathbf{\Delta}$', 'interpreter', 'latex');
legend({'Iteration 0','Iteration 1','Iteration 2','Iteration 3',...
    'Iteration 4','Iteration 5'}, 'interpreter', 'latex');
