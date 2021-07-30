classdef ParamNMPCDCLFDCBF < handle
    properties
        horizon
        horizonCLF
        horizonCBF
        alpha
        gamma
        P % weight for Lyapunov function
        xWeight
        uWeight
        sWeight
        omegaWeight
    end
    methods
        function self = ParamNMPCDCLFDCBF(horizon, horizon_clf, horizon_cbf, alpha, gamma, P, x_weight, u_weight, s_weight, omega_weight)
            self.horizon = horizon;
            self.horizonCLF = horizon_clf;
            self.horizonCBF = horizon_cbf;
            self.alpha = alpha;
            self.gamma = gamma;
            self.P = P;
            self.xWeight = x_weight;
            self.uWeight = u_weight;
            self.sWeight = s_weight;
            self.omegaWeight = omega_weight;
        end
    end
end