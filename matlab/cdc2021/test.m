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
total_time = 1.0;

% problem setup
simulator = CBFDT(system_param, x0, t0);

param_mpcdcbf = ParamMPCDCBF(6, 0.1, 10 * eye(3), 10.0 * eye(3), 1.0);
simulator.setOpt('mpcdcbf', param_mpcdcbf);
simulator.sim(total_time);

param_mpcgcbf = ParamMPCGCBF(6, 0.1, 10 * eye(3), 10.0 * eye(3), 1.0);
simulator.setOpt('mpcgcbf', param_mpcgcbf);
simulator.sim(total_time);

param_nmpcdcbf = ParamNMPCDCBF(6, 6, 0.1, 10 * eye(3), 10.0 * eye(3), 1.0, 10.0);
simulator.setOpt('nmpcdcbf', param_nmpcdcbf);
simulator.sim(total_time);

param_dclfdcbf = ParamDCLFDCBF(0.1, 0.1, 10 * eye(3), 1.0, 10.0);
simulator.setOpt('dclfdcbf', param_dclfdcbf);
simulator.sim(total_time);

param_nmpcdclfdcbf = ParamNMPCDCLFDCBF(6, 6, 6, 0.1, 0.1, 10.0 * eye(3), 10.0 * eye(3), 1.0, 10.0, 10.0);
simulator.setOpt('nmpcdclfdcbf', param_nmpcdclfdcbf);
simulator.sim(total_time);