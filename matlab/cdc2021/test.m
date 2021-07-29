close all
clear

% system setup and initial condition
timestep = 0.1;
system_param.A = [[1 ,timestep, 0]; [0, 1, timestep]; [0, 0, 1]];
system_param.B = [0; 0; timestep];
system_param.ul = -1;
system_param.uu = 1;
system_param.timestep = timestep;
x0 = [-2;0;1];
t0 = 0.0;
total_time = 5.0;

% problem setup
simulator = CBFDT(system_param, x0, t0);
% param_mpccbf = ParamMPCCBF(6, 0.1, 10 * eye(3), 10.0 * eye(3), 1.0);
% simulator.setOpt('mpccbf', param_mpccbf);
param_mpcgcbf = ParamMPCGCBF(6, 0.1, 10 * eye(3), 10.0 * eye(3), 1.0);
simulator.setOpt('mpcgcbf', param_mpcgcbf);
% param_cbfnmpc = ParamCBFNMPC(6, 6, 0.1, 10 * eye(3), 10.0 * eye(3), 1.0, 10.0);
% simulator.setOpt('cbfnmpc', param_cbfnmpc);
% param_dclfdcbf = ParamDCLFDCBF(0.1, 0.1, 10 * eye(3), 1.0, 10.0);
% simulator.setOpt('dclfdcbf', param_dclfdcbf);
% param_clfcbfnmpc = ParamCLFCBFNMPC(6, 6, 6, 0.1, 0.1, 10.0 * eye(3), 10.0 * eye(3), 1.0, 10.0, 10.0);
% simulator.setOpt('clfcbfnmpc', param_clfcbfnmpc);
simulator.sim(total_time);