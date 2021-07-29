classdef ParamDCLFDCBF < handle
    properties
        alpha
        gamma
        P % weight for Lyapunov function
        uWeight
        sWeight
    end
    methods
        function self = ParamDCLFDCBF(alpha, gamma, P, u_weight, s_weight)
            self.alpha = alpha;
            self.gamma = gamma;
            self.P = P;
            self.uWeight = u_weight;
            self.sWeight = s_weight;
        end
    end
end