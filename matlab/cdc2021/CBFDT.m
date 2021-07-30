classdef CBFDT < handle
    properties
        systemParam
        optType
        optParam
        xcurr
        timecurr
        solvertime
        xdim
        udim
        tlog = []
        xlog = []
        ulog = []
        omegalog = []
    end
    
    methods
        function self = CBFDT(system_param, x0, t0)
            self.systemParam = system_param;
            self.xcurr = x0;
            self.timecurr = t0;
            self.xdim = size(self.systemParam.A, 1);
            self.udim = size(self.systemParam.B, 2);
        end
        
        function setOpt(self, opt_type, opt_param)
            self.optType = opt_type;
            self.optParam = opt_param;
        end
        
        function sim(self, total_time)
            xk = self.xcurr;
            while self.timecurr <= total_time
                [feas, xopt, uopt, Jopt] = self.solve();
                if feas ~= 1
                    return;
                end
                uk = uopt(:,1);
                xk = self.systemParam.A * xk + self.systemParam.B * uk;
                self.xcurr = xk;
                self.timecurr = self.timecurr + self.systemParam.timestep;
                self.tlog = [self.tlog, self.timecurr];
                self.xlog = [self.xlog, xk];
                self.ulog = [self.ulog, uk];
            end
        end
        
        function [feas, xopt, uopt, Jopt] = solve(self)
            if strcmp(self.optType,'dclfdcbf')
                [feas, xopt, uopt, Jopt] = solveDCLFDCBF(self);
            elseif strcmp(self.optType,'nmpcdclfdcbf')
                [feas, xopt, uopt, Jopt] = solveNMPCDCLFDCBF(self);
            elseif strcmp(self.optType,'mpcdcbf')
                [feas, xopt, uopt, Jopt] = solveMPCDCBF(self);
            elseif strcmp(self.optType,'mpcgcbf')
                [feas, xopt, uopt, Jopt] = solveMPCGCBF(self);
            elseif strcmp(self.optType,'nmpcdcbf')
                [feas, xopt, uopt, Jopt] = solveNMPCDCBF(self);
            else
                disp('optimal control policy undefined');
            end
        end
        
        function [feas, xopt, uopt, Jopt] = solveDCLFDCBF(self)
            x = sdpvar(self.xdim, 2);
            u = sdpvar(self.udim, 1);
            s = sdpvar(1,1);
            cost = 0;
            constraints = [];
            % initial condition
            constraints = [constraints; x(:,1) == self.xcurr];
            % dynamics
            constraints = [constraints; x(:,2) == self.systemParam.A * x(:,1) + self.systemParam.B * u(:,1)];
            % DCLF constraint
            v = x(:,1)' * self.optParam.P * x(:,1);
            vnext = x(:,2)' * self.optParam.P * x(:,2);
            constraints = [constraints; vnext - v + self.optParam.alpha * v <= s];
            % DCBF constraint
            r = 1;
            b = x(:,1)' * x(:,1) - r^2;
            bnext = x(:,2)' * x(:,2) - r^2;
            constraints = [constraints; bnext - b + self.optParam.gamma * b >= 0];
            % input constraint
            constraints = [constraints; self.systemParam.ul <= u <= self.systemParam.uu];
            % cost
            cost = cost + u' * self.optParam.uWeight * u + s' * self.optParam.sWeight * s;
            opt_settings = sdpsettings('solver','ipopt','verbose',0);
            diagnostics = optimize(constraints, cost, opt_settings);
            if diagnostics.problem == 0
                feas = true;
                xopt = value(x);
                uopt = value(u);
                Jopt = value(cost);
            else
                feas = false;
                xopt= [];
                uopt = [];
                Jopt = value(cost);
            end
            self.solvertime = diagnostics.solvertime;
            fprintf('solver time: %f\n', diagnostics.solvertime);
        end
        
        function [feas, xopt, uopt, Jopt] = solveNMPCDCLFDCBF(self)
            x = sdpvar(self.xdim, self.optParam.horizon + 1);
            u = sdpvar(self.udim, self.optParam.horizon);
            s = sdpvar(1,self.optParam.horizonCLF);
            omega = sdpvar(1,self.optParam.horizonCBF);
            cost = 0;
            constraints = [];
            % initial condition
            constraints = [constraints; x(:,1) == self.xcurr];
            % MPC cost and constraints
            for i = 1:self.optParam.horizon
                constraints = [constraints;
                    self.systemParam.ul <= u(:,i) <= self.systemParam.uu;
                    x(:,i+1) == self.systemParam.A * x(:,i) + self.systemParam.B * u(:,i)];
                cost = cost + x(:,i)'*self.optParam.xWeight*x(:,i);
                cost = cost + u(:,i)'*self.optParam.uWeight*u(:,i);
            end
            cost = cost + x(:,self.optParam.horizon)' * self.optParam.P * x(:,self.optParam.horizon);
            % CLF constraints
            for i = 1:self.optParam.horizonCLF
                v = x(:,1)' * self.optParam.P * x(:,1);
                vnext = x(:,2)' * self.optParam.P * x(:,2);
                constraints = [constraints; vnext - v + self.optParam.alpha * v <= s(i)];
                cost = cost + s(i)' * self.optParam.sWeight * s(i);
            end
            % CBF constraints
            for i = 1:self.optParam.horizonCBF
                r = 1;
                b = x(:,1)' * x(:,1) - r^2;
                bnext = x(:,2)' * x(:,2) - r^2;
                constraints = [constraints; bnext >= omega(i) * (1 - self.optParam.gamma) * b];
                constraints = [constraints; omega(i) >= 0];
                assign(omega(i), 1);
                cost = cost + self.optParam.omegaWeight * (omega(i) - 1)^2;
            end
            opt_settings = sdpsettings('solver','ipopt','verbose',0);
            diagnostics = optimize(constraints, cost, opt_settings);
            if diagnostics.problem == 0
                feas = true;
                xopt = value(x);
                uopt = value(u);
                Jopt = value(cost);
            else
                feas = false;
                xopt= [];
                uopt = [];
                Jopt = value(cost);
            end
            self.solvertime = diagnostics.solvertime;
            fprintf('solver time: %f\n', diagnostics.solvertime);
        end
        
        function [feas, xopt, uopt, Jopt] = solveMPCDCBF(self)
            x = sdpvar(self.xdim, self.optParam.horizon + 1);
            u = sdpvar(self.udim, self.optParam.horizon);
            cost = 0;
            constraints = [];
            % initial condition
            constraints = [constraints; x(:,1) == self.xcurr];
            % MPC cost and constraints
            for i = 1:self.optParam.horizon
                constraints = [constraints;
                    self.systemParam.ul <= u(:,i) <= self.systemParam.uu;
                    x(:,i+1) == self.systemParam.A * x(:,i) + self.systemParam.B * u(:,i)];
                cost = cost + x(:,i)'*self.optParam.xWeight*x(:,i);
                cost = cost + u(:,i)'*self.optParam.uWeight*u(:,i);
            end
            cost = cost + x(:,self.optParam.horizon)' * self.optParam.P * x(:,self.optParam.horizon);
            % CBF constraints
            for i = 1:self.optParam.horizon
                r = 0;
                b = r - x(1,i);
                bnext = r - x(1,i+1);
                constraints = [constraints; bnext - b + self.optParam.gamma * b >= 0];
            end
            opt_settings = sdpsettings('solver','ipopt','verbose',0);
            diagnostics = optimize(constraints, cost, opt_settings);
            if diagnostics.problem == 0
                feas = true;
                xopt = value(x);
                uopt = value(u);
                Jopt = value(cost);
            else
                feas = false;
                xopt= [];
                uopt = [];
                Jopt = value(cost);
            end
            self.solvertime = diagnostics.solvertime;
            fprintf('solver time: %f\n', diagnostics.solvertime);
        end
        
        function [feas, xopt, uopt, Jopt] = solveMPCGCBF(self)
            x = sdpvar(self.xdim, self.optParam.horizon + 1);
            u = sdpvar(self.udim, self.optParam.horizon);
            cost = 0;
            constraints = [];
            % initial condition
            constraints = [constraints; x(:,1) == self.xcurr];
            % MPC cost and constraints
            for i = 1:self.optParam.horizon
                constraints = [constraints;
                    self.systemParam.ul <= u(:,i) <= self.systemParam.uu;
                    x(:,i+1) == self.systemParam.A * x(:,i) + self.systemParam.B * u(:,i)];
                cost = cost + x(:,i)'*self.optParam.xWeight*x(:,i);
                cost = cost + u(:,i)'*self.optParam.uWeight*u(:,i);
            end
            cost = cost + x(:,self.optParam.horizon)' * self.optParam.P * x(:,self.optParam.horizon);
            % GCBF constraints
            num_HO = 3;
            r = 0;
            b = r - x(1,1);
            bnext = r - x(1,1+num_HO);
            constraints = [constraints; bnext >= (1 - self.optParam.gamma)^num_HO * b];            
            opt_settings = sdpsettings('solver','ipopt','verbose',0);
            diagnostics = optimize(constraints, cost, opt_settings);
            if diagnostics.problem == 0
                feas = true;
                xopt = value(x);
                uopt = value(u);
                Jopt = value(cost);
            else
                feas = false;
                xopt= [];
                uopt = [];
                Jopt = value(cost);
            end
            self.solvertime = diagnostics.solvertime;
            fprintf('solver time: %f\n', diagnostics.solvertime);
        end
        
        function [feas, xopt, uopt, Jopt] = solveNMPCDCBF(self)
            x = sdpvar(self.xdim, self.optParam.horizon + 1);
            u = sdpvar(self.udim, self.optParam.horizon);
            omega = sdpvar(1,self.optParam.horizonCBF);
            cost = 0;
            constraints = [];
            % initial condition
            constraints = [constraints; x(:,1) == self.xcurr];
            % MPC cost and constraints
            for i = 1:self.optParam.horizon
                constraints = [constraints;
                    self.systemParam.ul <= u(:,i) <= self.systemParam.uu;
                    x(:,i+1) == self.systemParam.A * x(:,i) + self.systemParam.B * u(:,i)];
                cost = cost + x(:,i)'*self.optParam.xWeight*x(:,i);
                cost = cost + u(:,i)'*self.optParam.uWeight*u(:,i);
            end
            cost = cost + x(:,self.optParam.horizon)' * self.optParam.P * x(:,self.optParam.horizon);
            % CBF constraints
            for i = 1:self.optParam.horizonCBF
                r = 0;
                b = r - x(1,i);
                bnext = r - x(1,i+1);
                constraints = [constraints; bnext >= omega(i) * (1 - self.optParam.gamma) * b];
                constraints = [constraints; omega(i) >= 0];
                assign(omega(i), 1);
                cost = cost + self.optParam.omegaWeight * (omega(i) - 1)^2;
            end
            opt_settings = sdpsettings('solver','ipopt','verbose',0);
            diagnostics = optimize(constraints, cost, opt_settings);
            if diagnostics.problem == 0
                feas = true;
                xopt = value(x);
                uopt = value(u);
                Jopt = value(cost);
            else
                feas = false;
                xopt= [];
                uopt = [];
                Jopt = value(cost);
            end
            self.solvertime = diagnostics.solvertime;
            fprintf('solver time: %f\n', diagnostics.solvertime);
        end
    end
end

