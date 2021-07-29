clc
close all
clear
% This file tests safety performance for various approaches

%% Setup and simulations
timestep = 0.1;
system_param.A = [[1 ,timestep, 0]; [0, 1, timestep]; [0, 0, 1]];
system_param.B = [0; 0; timestep];
system_param.ul = -1;
system_param.uu = 1;
system_param.timestep = timestep;
x0 = [-2.0;0.0;1.0];
t0 = 0.0;

gammalist = [0.05, 0.10, 0.15, 0.2];

%% MPC-CBF simulator
mpccbf_simulators = {};
for index = 1:length(gammalist)
    mpccbf_simulators{index} = CBFDT(system_param, x0, t0);
    param_mpccbf = ParamMPCCBF(8, gammalist(index), 10.0*eye(3), 10.0*eye(3), 1.0);
    mpccbf_simulators{index}.setOpt('mpccbf', param_mpccbf);
    mpccbf_simulators{index}.sim(10.0);
end

%% MPC-GCBF simulator
mpcgcbf_simulators = {};
for index = 1:length(gammalist)
    mpcgcbf_simulators{index} = CBFDT(system_param, x0, t0);
    param_mpcgcbf = ParamMPCGCBF(8, gammalist(index), 10.0*eye(3), 10.0*eye(3), 1.0);
    mpcgcbf_simulators{index}.setOpt('mpcgcbf', param_mpcgcbf);
    mpcgcbf_simulators{index}.sim(10.0);
end

%% CBF-NMPC simulator
cbfnmpc_simulators = {};
for index = 1:length(gammalist)
    cbfnmpc_simulators{index} = CBFDT(system_param, x0, t0);
    param_cbfnmpc = ParamCBFNMPC(8, 8, gammalist(index), 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
    cbfnmpc_simulators{index}.setOpt('cbfnmpc', param_cbfnmpc);
    cbfnmpc_simulators{index}.sim(10.0);
end

%% Plotting
figure('Renderer', 'painters', 'Position', [0 0 500 400]);
color1 = '[0, 0.4470, 0.7410]';
color2 = '[0.8500, 0.3250, 0.0980]';
color3 = '[0.9290, 0.6940, 0.1250]';
color4 = '[0.4940, 0.1840, 0.5560]';
set(gca,'LineWidth', 0.2, 'FontSize', 20);
hold on;
grid on;
set(gca, 'YScale', 'log');
plot(mpccbf_simulators{2}.tlog, -mpccbf_simulators{2}.xlog(1, :), 'Color', color2, 'LineWidth', 1.0);
plot(mpccbf_simulators{3}.tlog, -mpccbf_simulators{3}.xlog(1, :), 'Color', color3, 'LineWidth', 1.0);
plot(mpccbf_simulators{4}.tlog, -mpccbf_simulators{4}.xlog(1, :), 'Color', color4, 'LineWidth', 1.0);
h_legend = legend('MPC-CBF ($\gamma=0.10$)', 'MPC-CBF ($\gamma=0.15$)', 'MPC-CBF ($\gamma=0.20$)');
set(h_legend, 'Interpreter','latex', 'Location', 'SouthWest');
xlim([0, 10]);
% save data and generate figures
print(gcf, 'figures/safety-mpccbf.eps', '-depsc');
print(gcf, 'figures/safety-mpccbf.png', '-dpng', '-r800');
save('data/safety-mpccbf.mat');

figure('Renderer', 'painters', 'Position', [0 0 500 400]);
set(gca,'LineWidth', 0.2, 'FontSize', 20);
hold on;
grid on;
set(gca, 'YScale', 'log');
plot(mpcgcbf_simulators{2}.tlog, -mpcgcbf_simulators{2}.xlog(1, :), 'Color', color2, 'LineWidth', 1.0);
plot(mpcgcbf_simulators{3}.tlog, -mpcgcbf_simulators{3}.xlog(1, :), 'Color', color3, 'LineWidth', 1.0);
plot(mpcgcbf_simulators{4}.tlog, -mpcgcbf_simulators{4}.xlog(1, :), 'Color', color4, 'LineWidth', 1.0);
h_legend = legend('MPC-GCBF ($\gamma=0.10$)', 'MPC-GCBF ($\gamma=0.15$)', 'MPC-GCBF ($\gamma=0.20$)');
set(h_legend, 'Interpreter','latex', 'Location', 'SouthWest');
xlim([0, 10]);
ylim([0.01, 2]);
% save data and generate figures
print(gcf, 'figures/safety-mpcgcbf.eps', '-depsc');
print(gcf, 'figures/safety-mpcgcbf.png', '-dpng', '-r800');
save('data/safety-mpcgcbf.mat');

figure('Renderer', 'painters', 'Position', [0 0 500 400]);
set(gca,'LineWidth', 0.2, 'FontSize', 20);
hold on;
grid on;
set(gca, 'YScale', 'log');
plot(cbfnmpc_simulators{1}.tlog, -cbfnmpc_simulators{1}.xlog(1, :), 'Color', color1, 'LineWidth', 1.0);
plot(cbfnmpc_simulators{2}.tlog, -cbfnmpc_simulators{2}.xlog(1, :), 'Color', color2, 'LineWidth', 1.0);
plot(cbfnmpc_simulators{3}.tlog, -cbfnmpc_simulators{3}.xlog(1, :), 'Color', color3, 'LineWidth', 1.0);
plot(cbfnmpc_simulators{4}.tlog, -cbfnmpc_simulators{4}.xlog(1, :), 'Color', color4, 'LineWidth', 1.0);
h_legend = legend('CBF-NMPC ($\gamma=0.05$)', 'CBF-NMPC ($\gamma=0.10$)', 'CBF-NMPC ($\gamma=0.15$)', 'CBF-NMPC ($\gamma=0.20$)');
set(h_legend, 'Interpreter','latex', 'Location', 'SouthWest');
xlim([0, 10]);
% save data and generate figures
print(gcf, 'figures/safety-cbfnmpc.eps', '-depsc');
print(gcf, 'figures/safety-cbfnmpc.png', '-dpng', '-r800');
save('data/safety-cbfnmpc.mat');

%% Comparison with different p_omega