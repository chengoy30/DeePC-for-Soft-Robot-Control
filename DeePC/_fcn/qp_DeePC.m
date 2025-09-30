function [u_opt,y_opt,problem_status] = qp_DeePC(Up,Yp,Uf,Yf,...
    uini,yini,Q,R,y_star,u_star,lambda_g,lambda_y,u_limit,y_limit)
% =========================================================================
%                   DeePC
% Input:
%   Up & Uf:             Hankel matrix of pre-collected input data
%   Yp & Yf:             Hankel matrix of pre-collected output data
%   uini & yini:         past data of length Tini in control process
%   Q & R:               weight coefficient matrix in performance cost
%   r:                   reference trajectory
%   lambda_g & lambda_y: coefficient in regulation for nonlinearty and uncertainty
%   u_limit & s_limit:   bound on control input & output
% Output:
%   u_opt:               designed optimal future control input
%   y_opt:               predicted output in the optimal future control input
%   problem_status:      problem status in optimization calculation
%
%                      Optimization Formulation
% mininize:
%   ||y||_{Q_blk}^2 + ||u||_{R_blk}^2 + lambda_g||g||_2^2 + lambda_y||sigma_y||_2^2
% subject to:
%   [Up]    [uini]   [   0   ]
%   [Yp]g = [yini] + [sigma_y], u in u_limit, y in y_limit
%   [Uf]    [ u  ]   [   0   ]
%   [Yf]    [ y  ]   [   0   ]
%
% We transform the problem into **standard quadratic programming** for calculation
% =========================================================================

% whether there exists input/output constraints
if nargin <= 12          
    constraint_bool = 0;
else
    constraint_bool = 1;
end

% ------------
% parameters
% ------------
m        = size(uini,1);                % dimension of control input
p        = size(yini,1);                % dimension of output
Tini     = size(Up,1)/m;                % horizon of past data
N        = size(Uf,1)/m;                % horizon of future data
T        = size(Up,2) + Tini + N - 1;   % time length of pre-collected data

% reshape past data into one single trajectory
% uini = col(u(-Tini),u(-Tini+1),...,u(-1)) (similarly for yini and eini)
uini_col = reshape(uini,[m*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
ur_col = reshape(u_star*ones(1,N),[m*N,1]);
yr_col = reshape(y_star*ones(1,N),[p*N,1]);
umin_col = reshape(u_limit(:,1)*ones(1,N),[m*N,1]);
umax_col = reshape(u_limit(:,2)*ones(1,N),[m*N,1]);
ymin_col = reshape(y_limit(:,1)*ones(1,N),[p*N,1]);
ymax_col = reshape(y_limit(:,2)*ones(1,N),[p*N,1]);

Q_blk    = zeros(p*N);
R_blk    = zeros(m*N); 
for i = 1:N
    Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
    R_blk((i-1)*m+1:i*m,(i-1)*m+1:i*m) = R; 
end

% ---------------------
% Standard QP in MATLAB
% [x,fval,exitflag,output,lambda]=quadprog(H,f,A,b,B,c,l,u,x0,options)
% minimize     0.5*x'*H*x+f'*x    
% subject to         A*x          <= b 
%                    B*x           = c
%                    l <= x <= u 
% ---------------------

% Coefficient
H       = Yf'*Q_blk*Yf + Uf'*R_blk*Uf + lambda_g*eye(T-Tini-N+1) + lambda_y*Yp'*Yp;
f       = -lambda_y*Yp'*yini_col-Yf'*Q_blk*yr_col;

if lambda_y == 0
    B       = [Up;Yp];
    c       = [uini_col;yini_col];
else
    B       = [Up];
    c       = [uini_col];
end

if constraint_bool % there exists input/output constraints
    A = [Uf;-Uf;Yf;-Yf];
    b = [umax_col;-umin_col;...
         ymax_col;-ymin_col];
else
    A = [];
    b = [];
end

% options = optimoptions('quadprog','MaxIterations',1e4);
options = optimoptions('quadprog','MaxIterations',1e3);
% Optimization
[g_opt,fval,exitflag,output,lambda] = quadprog(H,f,A,b,B,c,[],[],[],options);

% Solution
u_opt   = Uf*g_opt;
y_opt   = Yf*g_opt;
problem_status = exitflag;

% % For infeasible cases
% if exitflag ~= 1
%     u_opt = previous_u_opt;
% end

end