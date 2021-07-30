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
simulator_dt = CBFDT(system_param, x0, t0);
param_mpcdcbf = ParamMPCDCBF(10, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0);
simulator_dt.setOpt('mpcdcbf', param_mpcdcbf);
solvertime_list1 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with MPC-DCBF
            simulator_dt.xcurr = xfeas;
            [feas, ~, ~, ~] = simulator_dt.solve;
            if feas == 1
                solvertime_list1 = [solvertime_list1, simulator_dt.solvertime];
            end
        end
    end
end

%% MPC-GCBF (N=10)
solvertime_list2 = [];
simulator_dt = CBFDT(system_param, x0, t0);
param_mpcgcbf = ParamMPCGCBF(10, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0);
simulator_dt.setOpt('mpcgcbf', param_mpcgcbf);
solvertime_list2 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with MPC-GCBF
            simulator_dt.xcurr = xfeas;
            [feas, ~, ~, ~] = simulator_dt.solve;
            if feas == 1
                solvertime_list2 = [solvertime_list2, simulator_dt.solvertime];
            end
        end
    end
end

%% NMPC-DCBF (N=10, MCBF=10)
solvertime_list3 = [];
simulator_dt = CBFDT(system_param, x0, t0);
param_nmpcdcbf = ParamNMPCDCBF(10, 10, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
simulator_dt.setOpt('nmpcdcbf', param_nmpcdcbf);
solvertime_list3 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with NMPC-DCBF
            simulator_dt.xcurr = xfeas;
            [feas, ~, ~, ~] = simulator_dt.solve;
            if feas == 1
                solvertime_list3 = [solvertime_list3, simulator_dt.solvertime];
            end
        end
    end
end

%% NMPC-DCBF (N=10, MCBF=8)
solvertime_list4 = [];
simulator_dt = CBFDT(system_param, x0, t0);
param_nmpcdcbf = ParamNMPCDCBF(10, 8, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
simulator_dt.setOpt('nmpcdcbf', param_nmpcdcbf);
solvertime_list4 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with NMPC-DCBF
            simulator_dt.xcurr = xfeas;
            [feas, ~, ~, ~] = simulator_dt.solve;
            if feas == 1
                solvertime_list4 = [solvertime_list4, simulator_dt.solvertime];
            end
        end
    end
end

%% NMPC-DCBF (N=10, MCBF=5)
solvertime_list5 = [];
simulator_dt = CBFDT(system_param, x0, t0);
param_nmpcdcbf = ParamNMPCDCBF(10, 5, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
simulator_dt.setOpt('nmpcdcbf', param_nmpcdcbf);
solvertime_list5 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with NMPC-DCBF
            simulator_dt.xcurr = xfeas;
            [feas, ~, ~, ~] = simulator_dt.solve;
            if feas == 1
                solvertime_list5 = [solvertime_list5, simulator_dt.solvertime];
            end
        end
    end
end

%% NMPC-DCBF (N=10, MCBF=3)
solvertime_list6 = [];
simulator_dt = CBFDT(system_param, x0, t0);
param_nmpcdcbf = ParamNMPCDCBF(10, 3, 0.2, 10.0*eye(3), 10.0*eye(3), 1.0, 10.0);
simulator_dt.setOpt('nmpcdcbf', param_nmpcdcbf);
solvertime_list6 = [];
for k1 = 1:length(x1list)
    for k2 = 1:length(x2list)
        for k3 = 1:length(x3list)
            xfeas = [x1list(k1); x2list(k2); x3list(k3)];
            % solve the problem with NMPC-DCBF
            simulator_dt.xcurr = xfeas;
            [feas, ~, ~, ~] = simulator_dt.solve;
            if feas == 1
                solvertime_list6 = [solvertime_list6, simulator_dt.solvertime];
            end
        end
    end
end