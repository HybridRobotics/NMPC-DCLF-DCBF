classdef ParamMPCCBF < handle
    properties
        horizon
        gamma
        P % weight for terminal cost (CLF)
        xWeight
        uWeight
    end
    methods
        function self = ParamMPCCBF(horizon, gamma, P, x_weight, u_weight)
            self.horizon = horizon;
            self.gamma = gamma;
            self.P = P;
            self.xWeight = x_weight;
            self.uWeight = u_weight;
        end
    end
end