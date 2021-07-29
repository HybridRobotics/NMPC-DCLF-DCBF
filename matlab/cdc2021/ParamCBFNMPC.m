classdef ParamCBFNMPC < handle
    properties
        horizon
        horizonCBF
        gamma
        P % weight for terminal cost (CLF)
        xWeight
        uWeight
        omegaWeight
    end
    methods
        function self = ParamCBFNMPC(horizon, horizon_cbf, gamma, P, x_weight, u_weight, omega_weight)
            self.horizon = horizon;
            self.horizonCBF = horizon_cbf;
            self.gamma = gamma;
            self.P = P;
            self.xWeight = x_weight;
            self.uWeight = u_weight;
            self.omegaWeight = omega_weight;
        end
    end
end