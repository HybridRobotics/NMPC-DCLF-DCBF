classdef NMPCDCBF2 < handle
    % MPC with distance constraints
    properties
        system
        params
        x0
        x_curr
        time_curr = 0.0
        xlog = []
        ulog = []
        tt = 0
        distlog = []
        solvertime = []
        xopenloop = {}
        uopenloop = {}
        u_cost = 0
        obs
    end
    methods
        function self = NMPCDCBF2(x0, system, params)
            % Define MPC_CBF controller
            self.x0 = x0;
            self.x_curr = x0;
            self.system = system;
            self.params = params;
        end
        
        function sim(self, time)
            % Simulate the system until a given time
            tic;
            xk = self.x_curr;
            while self.time_curr < time
                [~, uk, tt] = self.solveNMPCDCBF1(self.x_curr);%if infeasiblility happens, stop
                if tt == -1
                    self.tt = tt;%record computing time for each time-step
                    return
                end
                xk = self.system.A * xk + [xk(4,1) * cos(xk(3,1))*self.system.dt;xk(4,1) * sin(xk(3,1))*self.system.dt;0;0] + self.system.C * uk;
                % update system
                self.x_curr = xk;
                self.time_curr = self.time_curr + self.system.dt;
                self.xlog = [self.xlog, xk];
                self.ulog = [self.ulog, uk];
                self.u_cost = self.u_cost + uk'*uk;
                tt = tt + toc;
                self.tt = tt;
                return
            end
            
        end
        
        function [xopt, uopt,tt] = solveNMPCDCBF1(self, xk)
            % Solve NMPC-DCBF
            [feas, x, u, J] = self.solve_cftoc1(xk);
            if ~feas
                xopt = [];
                uopt = [];
                tt = -1;
                return
            else
                xopt = x(:,2);
                uopt = u(:,1);
                tt = 0;
            end
        end
        
        function [feas, xopt, uopt, Jopt] = solve_cftoc1(self, xk)
            % Solve CFTOC
            % extract variables
            N = self.params.N;
            % define variables and cost
            x = sdpvar(4, N+1);%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            u = sdpvar(4, N);
            constraints = [];
            cost = 0;
            % initial constraint
            constraints = [constraints; x(:,1) == xk];
            % add constraints and costs
            AA=self.system.A;
            BB=self.system.B;
            CC=self.system.C;
            constraints = [constraints;
                    self.system.xl <= x(:,1) <= self.system.xu;
                    self.system.ul <= u(:,1) <= self.system.uu;
                    x(:,2) == AA * x(:,1) + [x(4,1)*cos(x(3,1))*self.system.dt;x(4,1)*sin(x(3,1))*self.system.dt;0; 0] + CC * u(:,1)];
                cost = cost + (x(:,1)-[3;0.01;0;0])'*self.params.Q*(x(:,1)-[3;0.01;0;0]) + (u(:,1)-[0;0;1;1])'*self.params.R*(u(:,1)-[0;0;1;1]); % self.params.S*(w(:,1)-1)^2;
            for i = 2:1:N
                constraints = [constraints;
                    self.system.xl <= x(:,i) <= self.system.xu;
                    self.system.ul <= u(:,i) <= self.system.uu;
                    x(:,i+1) == AA * x(:,i) + [x(4,i)*cos(x(3,i))*self.system.dt;x(4,i)*sin(x(3,i))*self.system.dt;0; 0] + CC * u(:,i)];
                cost = cost + (x(:,i)-[3;0.01;0;0])'*self.params.Q*(x(:,i)-[3;0.01;0;0]) + (u(:,i)-[0;0;1;1])'*self.params.R*(u(:,i)-[0;0;1;1]);% self.params.S*(w(:,i)-1)^2;
            end
            % add CBF constraints
            for i = 1:N-1
                pos = self.obs.pos1;
                r = self.obs.r1 ;
                b = (x([1:2],i)-pos)'*((x([1:2],i)-pos))-r^2;
                b_next = (x([1:2],i+1)-pos)'*((x([1:2],i+1)-pos))-r^2;
                b_next_next = (x([1:2],i+2)-pos)'*((x([1:2],i+2)-pos))-r^2;                
                b1 = (b_next - b)/self.system.dt + self.params.gamma1/self.system.dt * (b);
                b1_next = (b_next_next - b_next)/self.system.dt + self.params.gamma1/self.system.dt * (b_next);
                constraints = [constraints; b1_next  >=u(4,i)*(1-self.params.gamma2)*b1;b_next  >= u(3,i)*(1-self.params.gamma1)*b];%Highest order mcbf=2
            end
            % add terminal cost
            cost = cost + (x(:,N+1)-[3;0.01;0;0])'*self.params.P*(x(:,N+1)-[3;0.01;0;0]);
            ops = sdpsettings('solver','ipopt','verbose',0);
            % solve optimization
            diagnostics = optimize(constraints, cost, ops);
            if diagnostics.problem == 0
                feas = true;
                xopt = value(x);
                uopt = value(u);
                Jopt = value(cost);
            else
                feas = false;
                xopt = [];
                uopt = [];
                Jopt = [];
            end
            pos = self.obs.pos1;
            r = self.obs.r1 ;
            self.distlog = [self.distlog, (r^2-(xk(1:2)-pos)'*(xk(1:2)-pos))];
            self.xopenloop{size(self.xopenloop,2)+1} = xopt;
            self.uopenloop{size(self.uopenloop,2)+1} = uopt;
            self.solvertime = [self.solvertime, diagnostics.solvertime];
            fprintf('solver time: %f\n', diagnostics.solvertime);
        end
    end
end