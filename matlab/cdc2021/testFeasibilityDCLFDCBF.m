clc
close all
clear
% This file tests feasibility between DCLF-DCBF and CLF-CBF-NMPC.

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
    
    % MPC-GCBF simulator
    simulator_dclfdcbf = CBFDT(system_param, x0, t0);
    param_mpcgcbf = ParamDCLFDCBF(0.1, gammalist(gammaindex), 10 * eye(3), 1.0, 10.0);
    simulator_dclfdcbf.setOpt('dclfdcbf', param_mpcgcbf);
    % CBF-NMPC simulator
    simulator_clfcbfnmpc = CBFDT(system_param, x0, t0);
    param_clfcbfnmpc = ParamCLFCBFNMPC(8, 8, 8, 0.1, gammalist(gammaindex), 10.0 * eye(3), 10.0 * eye(3), 1.0, 10.0, 10.0);
    simulator_clfcbfnmpc.setOpt('clfcbfnmpc', param_clfcbfnmpc);
    % iterate over states
    x1list = linspace(-2,0,11);
    x2list = linspace(0,2,11);
    x3list = linspace(0,2,11);
    % data collection
    feas_points_dclfdcbf = [];
    feas_points_clfcbfnmpc = [];
    infeas_points_all = [];
    
    %% Test feasibility with sampling states among controllers
    
    for k1 = 1:length(x1list)
        for k2 = 1:length(x2list)
            for k3 = 1:length(x3list)
                xfeas = [x1list(k1); x2list(k2); x3list(k3)];
                % exclude the point if it's always unsafe
                if xfeas' * xfeas <= 1
                    continue;
                end
                % solve the problem with MPC-GCBF
                simulator_dclfdcbf.xcurr = xfeas;
                [feas_dclfdcbf, ~, ~, ~] = simulator_dclfdcbf.solve;
                % solve the problem with CBF-NMPC
                simulator_clfcbfnmpc.xcurr = xfeas;
                [feas_clfcbfnmpc, ~, ~, ~] = simulator_clfcbfnmpc.solve;
                if feas_dclfdcbf == 1
                    feas_points_dclfdcbf = [feas_points_dclfdcbf, xfeas];
                end
                if feas_clfcbfnmpc == 1
                    feas_points_clfcbfnmpc = [feas_points_clfcbfnmpc, xfeas];
                end
                if feas_dclfdcbf == 0 && feas_clfcbfnmpc == 0
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
    scatter3(feas_points_dclfdcbf(1,:),feas_points_dclfdcbf(2,:),feas_points_dclfdcbf(3,:),...
        5, 'o', 'LineWidth', 1.0, 'MarkerEdgeColor', inner_color, 'MarkerFaceColor', inner_color);
    scatter3(feas_points_clfcbfnmpc(1,:),feas_points_clfcbfnmpc(2,:),feas_points_clfcbfnmpc(3,:),...
        20, 'o', 'LineWidth', 1.2, 'MarkerEdgeColor', outer_color);
    r = 1.0;
    [x,y,z] = sphere(50);
    x = x*r; y = y*r; z = z*r;
    lightGrey = 0.8*[1 1 1]; % It looks better if the lines are lighter
    ball_surface = surface(x,y,z,  'FaceColor', 'none', 'EdgeColor', [0.75, 0.75, 0], 'facealpha', 0.5);
    axis equal
    if gammaindex == 1
        h=get(gca,'Children');
        h_legend = legend(h([end, end-1]),...
            {'DCLF-DCBF', 'CLF-CBF-NMPC'}, 'Location', 'NorthEast');
        set(h_legend, 'Interpreter','latex');
    end
    set(gca,'LineWidth', 1.0, 'FontSize', 15);
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$v (m/s)$','interpreter','latex','FontSize',20);
    zlabel('$a (m/s^2)$','interpreter','latex','FontSize',20);
    xlim([-2, 0]);
    ylim([0, 2]);
    zlim([0, 2]);
    view(160, 4);
    grid on;
    figurename_eps = "feasibility-dclfdbcf" + gammaindex + ".eps";
    figurename_png = "feasibility-dclfdbcf" + gammaindex + ".png";
    dataname = "feasibility-dclfdbcf" + gammaindex + ".mat";
    % save data and generate figures
    print(gcf,strcat('figures/',figurename_eps), '-depsc');
    print(gcf,strcat('figures/',figurename_png), '-dpng', '-r800');
    save(strcat('data/',dataname));
end