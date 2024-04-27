clear;clc
%% load data, falcon
load('CSLP_solution_min_u_obs_240419.mat');
%% smooth states
tf = problem.tf;
states = problem.states;
controls = problem.controls;
outputs = problem.outputs;

time = linspace(0,tf,21);
for k = 1:13
    states_interp(k,:) = smooth_interp(states(k,:), 3);
end
time_interp = linspace(0, tf, length(states_interp));
%%
figure(1);
hold on
grid on
plot3(states_interp(1, :), states_interp(2, :), 10-states_interp(3, :),...
    'linewidth',1);
set(gca,'YDir','reverse');
% pbaspect([5,2,1])
xlabel('$X~(m)$', 'interpreter', 'latex')
ylabel('$Y~(m)$', 'interpreter', 'latex')
zlabel('$H~(m)$', 'interpreter', 'latex')
%%
figure(2);
subplot(4,4,1)
hold on; grid on
plot(time_interp, states_interp(1,:),'linewidth',1)
ylabel('$X~(m)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,2)
hold on; grid on
plot(time_interp, states_interp(2,:),'linewidth',1)
ylabel('$Y~(m)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,3)
hold on; grid on
plot(time_interp, 10-states_interp(3,:),'linewidth',1)
ylabel('$H~(m)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,4)
hold on; grid on
plot(time_interp, 57.3*states_interp(4,:),'linewidth',1)
ylabel('$\theta~(^\circ)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,5)
hold on; grid on
plot(time_interp, 57.3*states_interp(5,:),'linewidth',1)
ylabel('$\phi~(^\circ)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,6)
hold on; grid on
plot(time_interp, 57.3*states_interp(6,:),'linewidth',1)
ylabel('$\psi~(^\circ)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,7)
hold on; grid on
plot(time_interp, states_interp(7,:),'linewidth',1)
ylabel('$u_E~(m/s)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,8)
hold on; grid on
plot(time_interp, states_interp(8,:),'linewidth',1)
ylabel('$v_E~(m/s)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,9)
hold on; grid on
plot(time_interp, states_interp(9,:),'linewidth',1)
ylabel('$w_E~(m/s)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,10)
hold on; grid on
plot(time_interp, 57.3*states_interp(10,:),'linewidth',1)
ylabel('$\dot{\theta}~(^\circ/s)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,11)
hold on; grid on
plot(time_interp, 57.3*states_interp(11,:),'linewidth',1)
ylabel('$\dot{\phi}~(^\circ/s)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,12)
hold on; grid on
plot(time_interp, 57.3*states_interp(12,:),'linewidth',1)
ylabel('$\dot{\psi}~(^\circ/s)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,13)
hold on; grid on
plot(time, controls(1,:),'linewidth',1)
ylabel('$u_1~(-)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,14)
hold on; grid on
plot(time, controls(2,:),'linewidth',1)
ylabel('$u_2~(-)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,15)
hold on; grid on
plot(time, controls(3,:),'linewidth',1)
ylabel('$u_3~(-)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')

subplot(4,4,16)
hold on; grid on
plot(time, controls(4,:),'linewidth',1)
ylabel('$u_4~(-)$', 'interpreter', 'latex')
xlabel('$t~(s)$', 'interpreter', 'latex')
%% SLP iteration
figure(3);
subplot(1,2,1)
hold on
plot(0:1:24,[10;problem.history.J(1:24)],'linewidth',1)
xlabel('Iteration number', 'interpreter', 'latex');
ylabel('Cost value $E(t_f)$', 'interpreter', 'latex');


subplot(1,2,2)
hold on
plot(0:1:24,[2.54;problem.history.EQ_vio(1:24)],'linewidth',1)
xlabel('Iteration number', 'interpreter', 'latex');
ylabel('$\|\mathbf{e}\|_{\infty}$', 'interpreter', 'latex');
set(gca, 'YScale', 'log')
%% J and merit, rho and delta_max, delta_all
figure(4)
subplot(3,1,1);
hold on
plot(0:1:12, [10;problem.history.J(1:12)],'linewidth',1)
plot(0:1:12, [5930.8;problem.history.merit(1:12)],'linewidth',1)
xlabel('Iteration number', 'interpreter', 'latex');
ylabel('Cost values', 'interpreter', 'latex');
legend({'$J$','$J_{\rm total}$'}, 'interpreter', 'latex');

subplot(3,1,2);
hold on
plot(0:1:12, [nan;problem.history.rho(1:12)],'linewidth',1)
plot(0:1:12, [5;problem.history.delta_max(1:12)],'linewidth',1)
xlabel('Iteration number', 'interpreter', 'latex');
legend({'$\rho$','$\Delta_{\rm max}$'}, 'interpreter', 'latex');

subplot(3,1,3);
hold on
plot(1:1:22, [5*ones(1,22);problem.history.delta_all],'linewidth',1)
set(gca, 'YScale', 'log')
xlabel('Collocation node index', 'interpreter', 'latex');
ylabel('Trust region $\mathbf{\Delta}$', 'interpreter', 'latex');
legend({'Iter 0','Iter 1','Iter 2','Iter 3',...
    'Iter 4','Iter 5','Iter 6','Iter 7','Iter 8',...
    'Iter 9','Iter 10'}, 'interpreter', 'latex');