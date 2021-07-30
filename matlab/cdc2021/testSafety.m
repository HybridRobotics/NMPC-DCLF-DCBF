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

%% MPC-DCBF simulator
mpcdcbf_simulators = {};
for index = 1:length(gammalist)
    mpcdcbf_simulators{index} = CBFDT(system_param, x0, t0);
    param_mpcdcbf = ParamMPCDCBF(8, gammalist(index), 10.0*eye(3), 10.0*eye(3), 1.0);
    mpcdcbf_simulators{index}.setOpt('mpcdcbf', param_mpcdcbf);
    mpcdcbf_simulators{index}.sim(10.0);
end

%% MPC-GCBF simulator
mpcgcbf_simulators = {};
for index = 1:length(gammalist)
    mpcgcbf_simulators{index} = CBFDT(system_param, x0, t0);
    param_mpcgcbf = ParamMPCGCBF(8, gammalist(index), 10.0*eye(3), 10.0*eye(3), 1.0);
    mpcgcbf_simulators{index}.setOpt('mpcgcbf', param_mpcgcbf);
    mpcgcbf_simulators{index}.sim(10.0);
end

%% NMPC-DCBF simulator
nmpcdcbf_simulators = {};
for index = 1:length(gammalist)
    nmpcdcbf_simulators{index} = CBFDT(system_param, x0, t0);
    param_nmpcdcbf = ParamNMPCDCBF(8, 8, gammalist(index), 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
    nmpcdcbf_simulators{index}.setOpt('nmpcdcbf', param_nmpcdcbf);
    nmpcdcbf_simulators{index}.sim(10.0);
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
plot(mpcdcbf_simulators{2}.tlog, -mpcdcbf_simulators{2}.xlog(1, :), 'Color', color2, 'LineWidth', 1.0);
plot(mpcdcbf_simulators{3}.tlog, -mpcdcbf_simulators{3}.xlog(1, :), 'Color', color3, 'LineWidth', 1.0);
plot(mpcdcbf_simulators{4}.tlog, -mpcdcbf_simulators{4}.xlog(1, :), 'Color', color4, 'LineWidth', 1.0);
h_legend = legend('MPC-DCBF ($\gamma=0.10$)', 'MPC-DCBF ($\gamma=0.15$)', 'MPC-DCBF ($\gamma=0.20$)');
set(h_legend, 'Interpreter','latex', 'Location', 'SouthWest');
xlim([0, 10]);
% save data and generate figures
print(gcf, 'figures/safety-mpcdcbf.eps', '-depsc');
print(gcf, 'figures/safety-mpcdcbf.png', '-dpng', '-r800');
save('data/safety-mpcdcbf.mat');

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
plot(nmpcdcbf_simulators{1}.tlog, -nmpcdcbf_simulators{1}.xlog(1, :), 'Color', color1, 'LineWidth', 1.0);
plot(nmpcdcbf_simulators{2}.tlog, -nmpcdcbf_simulators{2}.xlog(1, :), 'Color', color2, 'LineWidth', 1.0);
plot(nmpcdcbf_simulators{3}.tlog, -nmpcdcbf_simulators{3}.xlog(1, :), 'Color', color3, 'LineWidth', 1.0);
plot(nmpcdcbf_simulators{4}.tlog, -nmpcdcbf_simulators{4}.xlog(1, :), 'Color', color4, 'LineWidth', 1.0);
h_legend = legend('NMPC-DCBF ($\gamma=0.05$)', 'NMPC-DCBF ($\gamma=0.10$)', 'NMPC-DCBF ($\gamma=0.15$)', 'NMPC-DCBF ($\gamma=0.20$)');
set(h_legend, 'Interpreter','latex', 'Location', 'SouthWest');
xlim([0, 10]);
% save data and generate figures
print(gcf, 'figures/safety-nmpcdcbf.eps', '-depsc');
print(gcf, 'figures/safety-nmpcdcbf.png', '-dpng', '-r800');
save('data/safety-nmpccbf.mat');
