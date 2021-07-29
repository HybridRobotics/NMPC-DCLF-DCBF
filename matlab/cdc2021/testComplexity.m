clc
close all
clear
% This file tests computational time for various approaches

%% System setup

timestep = 0.1;
system_param.A = [[1 ,timestep, 0]; [0, 1, timestep]; [0, 0, 1]];
system_param.B = [0; 0; timestep];
system_param.ul = -1;
system_param.uu = 1;
system_param.timestep = timestep;
x0 = [0;0;0]; % this value will be overrided
t0 = 0.0;

%% Sampling states

x1list = linspace(-2,0,11);
x2list = linspace(0,2,11);
x3list = linspace(0,2,11);

%% MPC-CBF (N=10)
solvertime_list1 = [];
simulator_mpccbf = CBFDT(system_param, x0, t0);
param_mpccbf = ParamMPCCBF(10, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0);
simulator_mpccbf.setOpt('mpccbf', param_mpccbf);
solvertime_list1 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with MPC-CBF
            simulator_mpccbf.xcurr = xfeas;
            [feas_mpccbf, ~, ~, ~] = simulator_mpccbf.solve;
            if feas_mpccbf == 1
                solvertime_list1 = [solvertime_list1, simulator_mpccbf.solvertime];
            end
        end
    end
end

%% MPC-GCBF (N=10)
solvertime_list2 = [];
simulator_cbfnmpc = CBFDT(system_param, x0, t0);
param_mpcgcbf = ParamMPCGCBF(10, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0);
simulator_cbfnmpc.setOpt('mpcgcbf', param_mpcgcbf);
solvertime_list2 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with MPC-GCBF
            simulator_cbfnmpc.xcurr = xfeas;
            [feas_mpcgcbf, ~, ~, ~] = simulator_cbfnmpc.solve;
            if feas_mpcgcbf == 1
                solvertime_list2 = [solvertime_list2, simulator_cbfnmpc.solvertime];
            end
        end
    end
end

%% CBF-NMPC (N=10, MCBF=10)
solvertime_list3 = [];
simulator_cbfnmpc = CBFDT(system_param, x0, t0);
param_cbfnmpc = ParamCBFNMPC(10, 10, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
simulator_cbfnmpc.setOpt('cbfnmpc', param_cbfnmpc);
solvertime_list3 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with CBF-NMPC
            simulator_cbfnmpc.xcurr = xfeas;
            [feas_cbfnmpc, ~, ~, ~] = simulator_cbfnmpc.solve;
            if feas_cbfnmpc == 1
                solvertime_list3 = [solvertime_list3, simulator_cbfnmpc.solvertime];
            end
        end
    end
end

%% CBF-NMPC (N=10, MCBF=8)
solvertime_list4 = [];
simulator_cbfnmpc = CBFDT(system_param, x0, t0);
param_cbfnmpc = ParamCBFNMPC(10, 8, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
simulator_cbfnmpc.setOpt('cbfnmpc', param_cbfnmpc);
solvertime_list4 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with CBF-NMPC
            simulator_cbfnmpc.xcurr = xfeas;
            [feas_cbfnmpc, ~, ~, ~] = simulator_cbfnmpc.solve;
            if feas_cbfnmpc == 1
                solvertime_list4 = [solvertime_list4, simulator_cbfnmpc.solvertime];
            end
        end
    end
end

%% CBF-NMPC (N=10, MCBF=5)
solvertime_list5 = [];
simulator_cbfnmpc = CBFDT(system_param, x0, t0);
param_cbfnmpc = ParamCBFNMPC(10, 5, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
simulator_cbfnmpc.setOpt('cbfnmpc', param_cbfnmpc);
solvertime_list5 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with CBF-NMPC
            simulator_cbfnmpc.xcurr = xfeas;
            [feas_cbfnmpc, ~, ~, ~] = simulator_cbfnmpc.solve;
            if feas_cbfnmpc == 1
                solvertime_list5 = [solvertime_list5, simulator_cbfnmpc.solvertime];
            end
        end
    end
end

%% CBF-NMPC (N=10, MCBF=3)
solvertime_list6 = [];
simulator_cbfnmpc = CBFDT(system_param, x0, t0);
param_cbfnmpc = ParamCBFNMPC(10, 3, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
simulator_cbfnmpc.setOpt('cbfnmpc', param_cbfnmpc);
solvertime_list6 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with CBF-NMPC
            simulator_cbfnmpc.xcurr = xfeas;
            [feas_cbfnmpc, ~, ~, ~] = simulator_cbfnmpc.solve;
            if feas_cbfnmpc == 1
                solvertime_list6 = [solvertime_list6, simulator_cbfnmpc.solvertime];
            end
        end
    end
end