clc
close all
clear
% This file tests feasibility between MPC-CBF and CBF-NMPC.

%% System setup

timestep = 0.1;
system_param.A = [[1 ,timestep, 0]; [0, 1, timestep]; [0, 0, 1]];
system_param.B = [0; 0; timestep];
system_param.ul = -1;
system_param.uu = 1;
system_param.timestep = timestep;
x0 = [0;0;0]; % this value will be overrided
t0 = 0.0;

gammalist = [0.05, 0.1, 0.15, 0.2];
for gammaindex = 1:length(gammalist)
    %% Problem setup with different hyperparameters
    
    % MPC-CBF simulator
    simulator_mpccbf = CBFDT(system_param, x0, t0);
    param_mpccbf = ParamMPCCBF(8, gammalist(gammaindex), 10.0*eye(3), 10.0*eye(3), 1.0);
    simulator_mpccbf.setOpt('mpccbf', param_mpccbf);
    % CBF-NMPC simulator
    simulator_cbfnmpc = CBFDT(system_param, x0, t0);
    param_cbfnmpc = ParamCBFNMPC(8, 8, gammalist(gammaindex), 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
    simulator_cbfnmpc.setOpt('cbfnmpc', param_cbfnmpc);
    % iterate over states
    x1list = linspace(-2,0,11);
    x2list = linspace(0,2,11);
    x3list = linspace(0,2,11);
    % data collection
    feas_points_mpccbf = [];
    feas_points_cbfnmpc = [];
    infeas_points_all = [];
    
    %% Test feasibility with sampling states among controllers
    
    for k1 = 1:length(x1list)
        for k2 = 1:length(x2list)
            for k3 = 1:length(x3list)
                xfeas = [x1list(k1); x2list(k2); x3list(k3)];
                % solve the problem with MPC-CBF
                simulator_mpccbf.xcurr = xfeas;
                [feas_mpccbf, ~, ~, ~] = simulator_mpccbf.solve;
                % solve the problem with CBF-NMPC
                simulator_cbfnmpc.xcurr = xfeas;
                [feas_cbfnmpc, ~, ~, ~] = simulator_cbfnmpc.solve;
                if feas_mpccbf == 1
                    feas_points_mpccbf = [feas_points_mpccbf, xfeas];
                end
                if feas_cbfnmpc == 1
                    feas_points_cbfnmpc = [feas_points_cbfnmpc, xfeas];
                end
                if feas_mpccbf == 0 && feas_cbfnmpc == 0
                    infeas_points_all = [infeas_points_all, xfeas];
                end
            end
        end
    end
    
    %% Plotting
    close all
    outer_color = [0.3010, 0.7450, 0.9330];
    inner_color = [1, 0, 0];
    figure('Renderer', 'painters', 'Position', [0 0 400 400]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    hold on;
    scatter3(feas_points_mpccbf(1,:),feas_points_mpccbf(2,:),feas_points_mpccbf(3,:), 5, 'o','LineWidth',1.0, 'MarkerEdgeColor',inner_color,'MarkerFaceColor',inner_color);
    scatter3(feas_points_cbfnmpc(1,:),feas_points_cbfnmpc(2,:),feas_points_cbfnmpc(3,:), 20, 'o','LineWidth',1.2, 'MarkerEdgeColor',outer_color);
    axis equal
    if gammaindex == 1
        h=get(gca,'Children');
        h_legend = legend(h([end, end-1]),...
            {'MPC-CBF', 'CBF-NMPC'}, 'Location', 'NorthEast');
        set(h_legend, 'Interpreter','latex');
    end
    set(gca,'LineWidth', 1.0, 'FontSize', 15);
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$v (m/s)$','interpreter','latex','FontSize',20);
    zlabel('$a (m/s^2)$','interpreter','latex','FontSize',20);
    xlim([-2, 0]);
    ylim([0, 2]);
    zlim([0, 2]);
    view(70, 4);
    grid on;
    figurename_eps = "feasibility-mpccbf" + gammaindex + ".eps";
    figurename_png = "feasibility-mpccbf" + gammaindex + ".png";
    dataname = "feasibility-mpccbf" + gammaindex + ".mat";
    % save data and generate figures
    print(gcf,strcat('figures/',figurename_eps), '-depsc');
    print(gcf,strcat('figures/',figurename_png), '-dpng', '-r800');
    save(strcat('data/',dataname));
end