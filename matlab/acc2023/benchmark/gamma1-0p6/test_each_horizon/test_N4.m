close all
clear

%------------------------------------------------------------------------------------------
nb = 1000;%Total number of initial states
nmpcd=zeros(1,nb);%Store computing time for one time-step for NMPC-DCBF
impcd=zeros(1,nb);%Store computing time for one time-step for iMPC-DCBF
nmpcinf=zeros(1,1);%Store infeasible rate for one time-step for NMPC-DCBF
impcinf=zeros(1,1);%Store infeasible rate for one time-step for iMPC-DCBF
i=4;%i is number of horozon
ts = 1;%Total time steps
load(gamma1_0p6\InitialStateData');%Load the generated 1000 initial states
[a, b, c, d]=compare(i,ts,nb,InitialMat);%Involves NMPC-DCBF and iMPC-DCBF
nmpcd(1,:)=a;
impcd(1,:)=b;
nmpcinf(1,1)=c;
impcinf(1,1)=d;
nmpcplot1=nmpcd(1,:);
nmpcplot1(nmpcplot1==-1)=[];%Eliminate infeasible one time-step trajectories
impcplot1=impcd(1,:);
impcplot1(impcplot1==-1)=[];%Eliminate infeasible one time-step trajectories

save('timecom4','nmpcplot1','impcplot1');
save('feasibility4','nmpcinf','impcinf');

distnmpc = fitdist(nmpcplot1','Normal');
distimpc = fitdist(impcplot1','Normal');
mu1=distnmpc.mu;%Get mean of sample of computing time for NMPC-DCBF
mu2=distimpc.mu;%Get mean of sample of computing time for iMPC-DCBF
sigma1=distnmpc.sigma;%Get variance of sample of computing time for NMPC-DCBF
sigma2=distimpc.sigma;%Get variance of sample of computing time for iMPC-DCBF


%Initialize atmosphere parameters
function [tnmpc, timpc, ratiotnmpc, ratiotimpc]=compare(N11,ttsim,samplen,InitialMat)
tnmpc=[];
timpc=[];
for i=1:samplen
N1=N11;
tsim=ttsim;
xini1=InitialMat(1,i);
yini1=InitialMat(2,i);
thetaini1=InitialMat(3,i);
vini1=InitialMat(4,i);
x01=[xini1;yini1;thetaini1;vini1];
x02=[xini1 yini1 thetaini1 vini1];
t1 = nmpcdcbf(x01, N1, tsim);
t2 = impcdcbf(x02, N1, tsim);
tindex=[N11 i];
disp(tindex);
tnmpc=[tnmpc t1];%Computing time for NMPC-DCBF
timpc=[timpc t2];%Computing time for iMPC-DCBF
end
nnmpc1 = length(tnmpc);
nimpc1 = length(timpc);
tnmpcs = tnmpc;
timpcs = timpc;
tnmpcs(tnmpcs==-1)=[];
timpcs(timpcs==-1)=[];
nnmpc2 = length(tnmpcs);
nimpc2 = length(timpcs);
ratiotnmpc = (nnmpc1-nnmpc2)/nnmpc1;%Infeasible rate for NMPC-DCBF
ratiotimpc = (nimpc1-nimpc2)/nimpc1;%Infeasible rate for iMPC-DCBF
end





function t1 = nmpcdcbf(x00, N1, ttsim)
%% General Flags
run_nmpc_dcbf_one = true;
run_nmpc_dcbf_multiple = true;

%% Setup and Parameters
x0 = x00;
dt = 0.1;
time_total = ttsim *dt;
P = zeros(4,4);%Weight matrix P
P(1,1) = 10; P(2,2) = 10; P(3,3) = 10;P(4,4) = 10;
Q = zeros(4,4);%Weight matrix Q
Q(1,1) = 10; Q(2,2) = 10; Q(3,3) = 10;Q(4,4) = 10;
R = zeros(4,4);%Weight matrix R
R(1,1) = 1; R(2,2) = 1;R(3,3) = 1000;R(4,4) = 1000;
%Variables range as below
xmin = [-10; -10; -10;-10];
xmax = [10; 10; 10;10];
umin = [-7; -5;-inf;-inf];
umax = [7; 5;inf;inf];

%% Discrete-time unicycle model
system.dt = dt;
system.A = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
system.B = [x0(4,1)*cos(x0(3,1))*dt;
    x0(4,1)*sin(x0(3,1))*dt;
    0;
    0];
system.C =[0 0 0 0;
    0 0 0 0;
    1*dt 0 0 0;
    0 1*dt 0 0];
system.xl = xmin;
system.xu = xmax;
system.ul = umin;
system.uu = umax;

%% NMPC-DCBF parameters
params_nmpc_dcbf.Q = Q;
params_nmpc_dcbf.R = R;
params_nmpc_dcbf.P = P;
params_nmpc_dcbf.gamma1 = 0.6;
params_nmpc_dcbf.gamma2 = 0.6;

%% Obstacle
obs.pos1 = [0; 0];
obs.r1 = 1;



%% Simulate NMPC-DCBF with other N
params_nmpc_dcbf.N = N1;
if run_nmpc_dcbf_one
    fprintf('Run NMPC-DCBF-4\n');
    controller_nmpc_dcbf_multiple4 = NMPCDCBF1(x0, system, params_nmpc_dcbf);%Highest order mcbf=2
    controller_nmpc_dcbf_multiple4.obs = obs;
    controller_nmpc_dcbf_multiple4.sim(time_total);
end
t1=controller_nmpc_dcbf_multiple4.tt;
end


function t2=impcdcbf(x00, N1, ttsim)
t2=0;
xo = 0;
yo = 0;
r = 1;
N = N1;%Number of horizon
K1 = 1000;%Maximum iteration times, jmax
K = 1000;%Maximum iteration times, jmax
dt = 0.1;
gamma1 = 6;
gamma2 = 6;
nsim = ttsim;

tic;

% Constraints

umin = [-7; -5; -inf; -inf;];
umax = [7; 5; Inf; Inf];
xmin = [-10; -10; -10; -10];
xmax = [ 10;  10;  10;  10];

% Objective function
Q = diag([10 10 10 10]);
QN = Q;
R = 1 * eye(4);
R(3,3) = 1000;
R(4,4) = 1000;
% Initial and reference states
x0 = x00;
xr = [3; 0.01; 0; 0];
ur = [0; 0; 1; 1];

% Dynamic system initialization
BB = [0 0 0 0;0 0 0 0;dt 0 0 0;0 dt 0 0];

% Convex MPC frame
[nx, nu] = size(BB);
% Initialize states set (x00,x01,,,x0N）
x_0 = [];
x0 = x0';
u_0 = zeros(nu, N);
u0 = zeros(nu, 1);% we may need to change this for a better warm-up start
impc = zeros(nx, nsim + 1);
impc(:, 1) = x0;
x0new = x0;
for i=1 : (N+1)
    x_0 = [x_0 x0new];
    x0new = [dt*x0new(4)*cos(x0new(3))+x0new(1);dt*x0new(4)*sin(x0new(3))+x0new(2);dt*u0(1)+x0new(3);dt*u0(2)+x0new(4)];
end

abc = tangentline(x_0, r);
Acbf = cbfmat(abc, nx, nu, dt, gamma1, x0);%First order CBF constraints
lcbf = -abc(3, :);
lcbf(1) = [];
lcbf = lcbf';
ucbf = inf * ones(N, 1);
A2cbf = cbfmat2(abc, nx, nu, dt, gamma1, gamma2, x0);%Second order CBF constraints
lcbf2 = getlcbf2(abc, dt, gamma1, gamma2);
ucbf2 = inf * ones(N-1, 1);

% Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
% - quadratic objective
P = blkdiag( kron(speye(N), Q), QN, kron(speye(N), R) );
% - linear objective
q = [repmat(-Q*xr, N, 1); -QN*xr; repmat(-R*ur, N, 1)];
% - linear dynamics
Ax = getax(x_0, dt, N, nx);
Bu = kron([sparse(1, N); speye(N)], BB);
Cx = getcx(x_0, u_0, dt, N, nx);
Aeq = [Ax, Bu];
leq = [-x0; -Cx];
ueq = leq;
% - input and state constraints
Aineq = speye((N+1)*nx + N*nu);
lineq = [repmat(xmin, N+1, 1); repmat(umin, N, 1)];
uineq = [repmat(xmax, N+1, 1); repmat(umax, N, 1)];

% - OSQP constraints
A = [Aeq; Aineq; Acbf; A2cbf];
l = [leq; lineq; lcbf; lcbf2];
u = [ueq; uineq; ucbf; ucbf2];

% Create an OSQP object
prob = osqp;
% Setup workspace
prob.setup(P, q, A, l, u, 'warm_start', true);
% Solve
res = prob.solve();

% Check solver status
if ~strcmp(res.info.status, 'solved')
%    error('OSQP did not solve the problem!')
        t2=-1;
        return
end
ctrlx = res.x(1:(N+1)*nx);
ctrlu = res.x((N+1)*nx+1:(N+1)*nx+N*nu);

for j = 1 : (K1-1)%Iterations for the first time-step
    storagex = ctrlx;%store the state variables and inputs in order to compare them with the next iteration
    storageu = ctrlu;
    x_0 = trans(x0, ctrlx, nx, N+1);
    u_0 = transu(ctrlu, nu, N);
    abc = tangentline(x_0, r);
    Acbf = cbfmat(abc, nx, nu, dt, gamma1, x0);
    lcbf = -abc(3, :);
    lcbf(1) = [];
    lcbf = lcbf';
    ucbf = inf * ones(N, 1);
    A2cbf = cbfmat2(abc, nx, nu, dt, gamma1, gamma2, x0);
    lcbf2 = getlcbf2(abc, dt, gamma1, gamma2);
    ucbf2 = inf * ones(N-1, 1);
    % Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    % - quadratic objective
    P = blkdiag( kron(speye(N), Q), QN, kron(speye(N), R) );
    % - linear objective
    q = [repmat(-Q*xr, N, 1); -QN*xr; repmat(-R*ur, N, 1)];
    % - linear dynamics
    Ax = getax(x_0, dt, N, nx);
    Bu = kron([sparse(1, N); speye(N)], BB);
    Cx = getcx(x_0, u_0, dt, N, nx);
    Aeq = [Ax, Bu];
    leq = [-x0; -Cx];
    ueq = leq;
    % - input and state constraints
    Aineq = speye((N+1)*nx + N*nu);
    lineq = [repmat(xmin, N+1, 1); repmat(umin, N, 1)];
    uineq = [repmat(xmax, N+1, 1); repmat(umax, N, 1)];

    % - OSQP constraints
    A = [Aeq; Aineq; Acbf; A2cbf];
    l = [leq; lineq; lcbf; lcbf2];
    u = [ueq; uineq; ucbf; ucbf2];
    % Create an OSQP object
    prob = osqp;
    % Setup workspace
    prob.setup(P, q, A, l, u, 'warm_start', true);
    % Solve
    res = prob.solve();

    % Check solver status
    if ~strcmp(res.info.status, 'solved')
%        error('OSQP did not solve the problem!')
        t2=-1;
        return
    end
    ctrlx = res.x(1:(N+1)*nx);
    ctrlu = res.x((N+1)*nx+1:(N+1)*nx+N*nu);
    testx = (storagex - ctrlx)'*(storagex - ctrlx);
    testu = (storageu - ctrlu)'*(storageu - ctrlu);
    test = (testx)/(storagex'*storagex);
    if (test)^(0.5)<=10^(-2)&& (testx/((N+1)*nx))^(0.5)<=10^(-4)%Convergence criterion
      break
    end
end % Move for the first step
ctrl = ctrlu(1:nu);
x0 = [dt*x0(4)*cos(x0(3))+x0(1);dt*x0(4)*sin(x0(3))+x0(2);dt*ctrl(1)+x0(3);dt*ctrl(2)+x0(4)];
ctrlu = rewrite(ctrlu, nu, N);
t2=t2+toc;
return

x_0 = newinit(x0, ctrlu, nx, nu, N, dt);
u_0 = transu(ctrlu, nu, N);
impc(:, 2) = x0;
storagex = ctrlx;
storageu = ctrlu;

for i = 1 : (nsim-1)%Iterations for the left time steps

   for j = 1 : K
    abc = tangentline(x_0, r);
    Acbf = cbfmat(abc, nx, nu, dt, gamma1, x0);
    lcbf = -abc(3, :);
    lcbf(1) = [];
    lcbf = lcbf';
    ucbf = inf * ones(N, 1);
    A2cbf = cbfmat2(abc, nx, nu, dt, gamma1, gamma2, x0);
    lcbf2 = getlcbf2(abc, dt, gamma1, gamma2);
    ucbf2 = inf * ones(N-1, 1);
    % Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    % - quadratic objective
    P = blkdiag( kron(speye(N), Q), QN, kron(speye(N), R) );
    % - linear objective
    q = [repmat(-Q*xr, N, 1); -QN*xr; repmat(-R*ur, N, 1)];
    % - linear dynamics
    Ax = getax(x_0, dt, N, nx);
    Bu = kron([sparse(1, N); speye(N)], BB);
    Cx = getcx(x_0, u_0, dt, N, nx);
    Aeq = [Ax, Bu];
    leq = [-x0; -Cx];
    ueq = leq;
    % - input and state constraints
    Aineq = speye((N+1)*nx + N*nu);
    lineq = [repmat(xmin, N+1, 1); repmat(umin, N, 1)];
    uineq = [repmat(xmax, N+1, 1); repmat(umax, N, 1)];

    % - OSQP constraints
    A = [Aeq; Aineq; Acbf; A2cbf];
    l = [leq; lineq; lcbf; lcbf2];
    u = [ueq; uineq; ucbf; ucbf2];
    % Create an OSQP object
    prob = osqp;
    % Setup workspace
    prob.setup(P, q, A, l, u, 'warm_start', true);
    % Solve
    res = prob.solve();

    % Check solver status
    if ~strcmp(res.info.status, 'solved')
%        error('OSQP did not solve the problem!')
        t2=-1;
        return
    end
    ctrlx = res.x(1:(N+1)*nx);
    ctrlu = res.x((N+1)*nx+1:(N+1)*nx+N*nu);
    x0 = ctrlx(1:nx);
    testx0 = [testx0 x0];
    x_0 = trans(x0, ctrlx, nx, N+1);
    u_0 = transu(ctrlu, nu, N);
    testx = (storagex - ctrlx)'*(storagex - ctrlx);
    testu = (storageu - ctrlu)'*(storageu - ctrlu);
    test = (testx)/(storagex'*storagex);
    if (test)^(0.5)<=10^(-2)&& (testx/((N+1)*nx))^(0.5)<=10^(-4)%Convergence criterion
      break
    end
    storagex = ctrlx;
    storageu = ctrlu;
   end
   ctrl = ctrlu(1:nu);
   x0 = [dt*x0(4)*cos(x0(3))+x0(1);dt*x0(4)*sin(x0(3))+x0(2);dt*ctrl(1)+x0(3);dt*ctrl(2)+x0(4)];
   ctrlu = rewrite(ctrlu, nu, N);
   x_0 = newinit(x0, ctrlu, nx, nu, N, dt);
   u_0 = transu(ctrlu, nu, N);
   impc(:, i+2) = x0;
end
end


% Linerize the CBF constraints (get a, b, c for lines)
function abc = tangentline(xy, r)% x and y from initialize states set, abc are coeeficients for linear equation a*x+b*y+c=0
[xx, ~] = size(xy);%xx=2,yy=N+1
xy(xx,:) = []; % this part should be changed for other case
xy((xx-1),:) = []; % this part should be changed other case
[xx, yy] = size(xy);%xx=2,yy=N+1
xyjiao = zeros(xx, yy);%intersection points
for i = 1 : xx
    for j = 1 : yy
        xyjiao(i, j) = r * xy(i, j) * (1 / (xy(:, j)' * xy(:, j)))^(0.5);%calculate coordinates of intersection points
    end
end
cc = -r^2 * ones(1, yy);
abc = [xyjiao; cc];
end

% Get CBF constraints matrix 
function Acbf = cbfmat(abc, nx, nu, dt, gamma, x0)
[~, yy] = size(abc);
Acbfx = zeros((yy-1), yy*nx);
Acbfu = zeros((yy-1), (yy-1)*nu);
for i = 1 : (yy-1)
    Acbfx(i, (i*nx)+1) = abc(1, (i+1));
    Acbfx(i, (i*nx)+2) = abc(2, (i+1));
end
for i = 1 : (yy-1)
    Acbfu(i, ((i-1)*nu+3)) = - (1 - dt * gamma)^(i) * (abc(1, 1) * x0(1, 1) + abc(2, 1) * x0(2, 1) + abc(3, 1));
end
Acbf = [Acbfx Acbfu];
end

% Transfer vector x into matrix 
function res = trans(x0, vector, nxx, nyy)%nxx=nx,nyy=N+1
 res = zeros(nxx, nyy);
 res(:,1) = x0;
 for i = 1 : (nyy -1)
     res(:,i+1)= vector(((i)*nxx+1):(i+1)*nxx);
 end   
end

% Transfer vector u into matrix 
function resu = transu(vector, nxx, nyy)%nxx=nu,nyy=N
 resu = zeros(nxx, nyy);
 for i = 1 : (nyy)
     resu(:,i)= vector(((i-1)*nxx+1):(i)*nxx);
 end   
end

% Rewrite u vector
function reu = rewrite(vector, nu, N)
append = vector((N-1)*nu+1:N*nu);
vector(1:nu) = [];
reu = [vector;append];
end

% Get new x_0
function x_0 = newinit(x0, ctrlu, nx, nu, N, dt)
 x_0 = zeros(nx, N+1);
 x_0(:, 1) = x0;
 for i=1 : N
    u0 = ctrlu((i-1)*nu+1:i*nu);
    x0 = [dt*x0(4)*cos(x0(3))+x0(1);dt*x0(4)*sin(x0(3))+x0(2);dt*u0(1)+x0(3);dt*u0(2)+x0(4)];
    x_0(:, i + 1) = x0;
 end
end

% Get AA matrix
function AA = getaa(x0, dt)
 AA = [1 0 -x0(4)*sin(x0(3))*dt cos(x0(3))*dt;0 1 x0(4)*cos(x0(3))*dt sin(x0(3))*dt;0 0 1 0;0 0 0 1];
end
% Get CC matrix
function CC = getcc(x0, x1, u0, dt)
 CC = [x0(4)*sin(x0(3))*x0(3)*dt-x0(4)*cos(x0(3))*dt+x1(1)-x0(1);-x0(4)*cos(x0(3))*x0(3)*dt-x0(4)*sin(x0(3))*dt+x1(2)-x0(2);-u0(1)*dt+x1(3)-x0(3);-u0(2)*dt+x1(4)-x0(4)];
end
% Get Ax matrix
function Ax = getax(x_0, dt, N, nx)
x0 = x_0(:,1);
AA = getaa(x0, dt);
Ax = kron(speye(N+1), -speye(nx)) + kron(sparse(diag(ones(N, 1), -1)), AA);
for i = 1 : (N-1)
   x0 = x_0(:,i+1);
   AA = getaa(x0, dt);
   Ax(nx*(i+1)+1:nx*(i+1)+nx,nx*i+1:nx*i+nx) = AA;
end
end
% Get Cx matrix
function Cx = getcx(x_0, u_0, dt, N, nx)
Cx = zeros(N*nx, 1);
for i = 1 : N
    u0 = u_0(:,i);
    x0 = x_0(:,i);
    x1 = x_0(:,i+1);
    CC = getcc(x0, x1, u0, dt);
    Cx((i-1)*nx+1:(i-1)*nx+nx) = CC;
end
end
% Get A2cbf
function A2cbf = cbfmat2(abc, nx, nu, dt, gamma1, gamma2, x0)
[~, yy] = size(abc);
Acbfx2 = zeros((yy-2), yy*nx);
Acbfx22 = zeros((yy-2), yy*nx);
Acbfu2 = zeros((yy-2), (yy-1)*nu);
for i = 1 : (yy-2)
    Acbfx2(i, (i*nx)+1) = (gamma1-1/dt)*abc(1, (i+1));
    Acbfx2(i, (i*nx)+2) = (gamma1-1/dt)*abc(2, (i+1));
    Acbfx2(i, ((i+1)*nx)+1) = (1/dt)*abc(1, (i+2));
    Acbfx2(i, ((i+1)*nx)+2) = (1/dt)*abc(2, (i+2));
end
for i = 1 : (yy-2)
    Acbfx22(i, (1*nx)+1) = -(1-dt*gamma2)^(i)/dt*abc(1, (1+1));
    Acbfx22(i, (1*nx)+2) = -(1-dt*gamma2)^(i)/dt*abc(2, (1+1));
end
Acbfx2 = Acbfx2 + Acbfx22;
for i = 1 : (yy-2)
    Acbfu2(i, ((i-1)*nu+4)) = - (1 - dt * gamma2)^(i)*(gamma1-1/dt)*(abc(1, 1) * x0(1, 1) + abc(2, 1) * x0(2, 1) + abc(3, 1));
end
A2cbf = [Acbfx2 Acbfu2];
end
% Get lcbf2
function lcbf2 = getlcbf2(abc, dt, gamma1, gamma2)
[~, yy] = size(abc);
lcbf2 = zeros((yy-2),1);
for i = 1 : (yy-2)
    lcbf2(i, 1) = -abc(3, (i+2))/dt-(gamma1-1/dt)*abc(3, (i+1))+(1 - dt * gamma2)^(i)*abc(3, (1+1))/dt;
end
end