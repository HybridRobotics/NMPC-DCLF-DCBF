close all
clear

%--------------------------------------iMPC-DCBF------------------------------------------------
xo = 0;
yo = 0;
r = 1;
N = 24;%Number of horizon
K1 = 1000;%Maximum iteration times, jmax
K = 1000;%Maximum iteration times, jmax
dt = 0.1;
gamma1 = 4;
gamma2 = 4;
nsim = 100;%Total time steps
Itera_x = [];%This is to store iterative convergence of location x
Itera_y = [];%This is to store iterative convergence of location y
Itera_theta = [];%This is to store iterative convergence of orientation theta
Itera_v = [];%This is to store iterative convergence of speed v

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
x0 = [-3 0 0 0];
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
A = [Aeq; Aineq; Acbf; A2cbf];%Highest order mcbf=2
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
   error('OSQP did not solve the problem!')
end
ctrlx = res.x(1:(N+1)*nx);
ctrlu = res.x((N+1)*nx+1:(N+1)*nx+N*nu);

for j = 1 : (K1-1)
    storagex = ctrlx;
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
    A = [Aeq; Aineq; Acbf; A2cbf];%Highest order mcbf=2
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
       error('OSQP did not solve the problem!')
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
x_0 = newinit(x0, ctrlu, nx, nu, N, dt);
u_0 = transu(ctrlu, nu, N);
impc(:, 2) = x0;
storagex = ctrlx;
storageu = ctrlu;

for i = 1 : (nsim-1)

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
    A = [Aeq; Aineq; Acbf; A2cbf];%Highest order mcbf=2
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
       error('OSQP did not solve the problem!')
    end
    ctrlx = res.x(1:(N+1)*nx);
    ctrlu = res.x((N+1)*nx+1:(N+1)*nx+N*nu);
    if i == 6 %Get several open-loop trajectories at different iterations predicted at t = 6
     Itera_x = [Itera_x; ctrlx(1:4:(N*nx)+1)'];
     Itera_y = [Itera_y; ctrlx(2:4:(N*nx)+2)'];
     Itera_theta = [Itera_theta; ctrlx(3:4:(N*nx)+3)'];
     Itera_v = [Itera_v; ctrlx(4:4:(N*nx)+4)'];
    end
    x0 = ctrlx(1:nx);
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

save('trajectory','impctra');%Close-loop 2D-trajectory where tsim=100
save('iteration_x','Itera_x');
save('iteration_y','Itera_y');
save('iteration_theta','Itera_theta');
save('iteration_v','Itera_v');

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