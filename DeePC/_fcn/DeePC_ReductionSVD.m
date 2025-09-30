function [cost, tsim] = DeePC_ReductionSVD(L)
addpath('_fcn');
warning off;
rng(9);

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

% Linear Model
A = [0.921 0 0.041 0;...
     0 0.918 0 0.033;...
     0 0 0.924 0;...
     0 0 0 0.937];
B = [0.017 0.001;...
     0.001 0.023;...
     0 0.061;...
     0.072 0]; 
C = [1 0 0 0;...
     0 1 0 0]; 
 
n_ctr = size(A,2);
m_ctr = size(B,2);
p_ctr = size(C,1); 

% iteration
total_step     = 200;
% measurement noise
noise = 0.002;

% Desired setpoint
u_star              = [1;1];         % Desired input
y_star              = [0.65;0.77];   % Desired output

% Performance cost
weight_y            = 3;     % weight coefficient for spacing error   
weight_u            = 1e-4*eye(m_ctr);  % weight coefficient for control input
% Setup in DeeP-LCC
lambda_g            = 100;  % penalty on ||g||_2^2 in objective
lambda_y            = 1e4;  % penalty on ||sigma_y||_2^2 in objective
% Constraints
u_limit             = [-2,2;...
                       -2,2];
y_limit             = [-2,2;...
                       -2,2];

Q           = weight_y*eye(p_ctr);              % penalty for trajectory error
R           = weight_u*eye(m_ctr);              % penalty for control input
Lambda_y    = lambda_y*eye(p_ctr);


u           = zeros(m_ctr,total_step);     % control input
x           = zeros(n_ctr,total_step);     % state variables
y           = zeros(p_ctr,total_step);     % output variables
pr_status   = zeros(total_step,1);         % problem status

% ----------------
% Pre-collected data
% ----------------
% load pre-collected data for DeeP-LCC
i_data              = 2;    % id of the pre-collected data
load(['_data\trajectory_data_collection\data','_',num2str(i_data),'_noiseLevel_',num2str(noise),'_dataLength_1000','.mat']);
% iMa = 1: hankel matrix
iMa = 1;
% Horizon setup 
Tini                = 30;   % length of past data in control process
N                   = 45;   % length of future data in control process
n_length = 800;
if iMa == 1
    U  = hankel_matrix(ud,Tini+N);
    Up = U(1:Tini*m_ctr,1:n_length);
    Uf = U((Tini*m_ctr+1):end,1:n_length);
    Y  = hankel_matrix(yd,Tini+N);
    Yp = Y(1:Tini*p_ctr,1:n_length);
    Yf = Y((Tini*p_ctr+1):end,1:n_length);
else
    U_page  = page_matrix(ud,Tini+N);
    Up = U_page(1:Tini*m_ctr,1:n_length);
    Uf = U_page((Tini*m_ctr+1):end,1:n_length);
    Y_page  = page_matrix(yd,Tini+N);
    Yp = Y_page(1:Tini*p_ctr,1:n_length);
    Yf = Y_page((Tini*p_ctr+1):end,1:n_length);
end

AA = [Up;Yp;Uf;Yf];
[U_aa,S_aa,V_aa] = svd(AA);
AA_L = AA*V_aa(:,1:L);
Up = AA_L(1:size(Up,1),:);
Yp = AA_L(size(Up,1)+1:size(Up,1)+size(Yp,1),:);
Uf = AA_L(size(Up,1)+size(Yp,1)+1:size(Up,1)+size(Yp,1)+size(Uf,1),:);
Yf = AA_L(size(Up,1)+size(Yp,1)+size(Uf,1)+1:end,:);

% -------------------------------------------------------------------------
%   Simulation
%--------------------------------------------------------------------------

tic
% ------------------
%  Initialization
% ------------------
% initial past data in control process
uini = zeros(m_ctr,Tini);
yini = zeros(p_ctr,Tini);

x0 = [0.4;0.4;0.4;0.4];
x(:,1) = x0;
for k = 1:Tini-1    
    yini(:,k) = C*x(:,k)+noise*(-1+2*rand(p_ctr,1));    
    x(:,k+1) = A*x(:,k)+B*uini(:,k);  
end
k_end = k+1;
yini(:,k_end) = C*x(:,k_end)+noise*(-1+2*rand(p_ctr,1));

% update data in u,e,y
u(:,1:Tini) = uini;
y(:,1:Tini) = yini;

% For MPC, which might have infeasible cases
% previous_u_opt = 0; 
% previous_u_opt = zeros(N*n_cav,1);

% ------------------
%  Continue the simulation
% ------------------

for k = Tini:total_step-1
    
    [u_opt,y_opt,pr] = qp_DeePC(Up,Yp,Uf,Yf,uini,yini,Q,R,y_star,u_star,...
                        lambda_g,lambda_y,u_limit,y_limit);
%     [u_opt,y_opt,pr] = qp_DeePC_v2(Up,Yp,Uf,Yf,uini,yini,Q,R,y_star,u_star,...
%                         lambda_g,lambda_y,u_limit,y_limit);
%     [u_opt,y_opt,pr] = qp_DeePC_v3(Up,Yp,Uf,Yf,uini,yini,Q,R,y_star,u_star,...
%                         lambda_g,lambda_y,u_limit,y_limit,L);
    % one-step implementation in receding horizon manner
    u(:,k) = u_opt(1:m_ctr,1);     
    % record problem status
    pr_status(k) = pr;
    % update past data in control process
    uini = u(:,k-Tini+1:k);
    
    % update system state
    x(:,k+1) = A*x(:,k)+B*u(:,k);
    y(:,k+1) = C*x(:,k+1)+noise*(-1+2*rand(p_ctr,1)); 
    
    yini = y(:,k-Tini+1:k);
  
    fprintf('Simulation number: %d  |  process... %2.2f%% \n',i_data,(k-Tini)/total_step*100);
    %fprintf('Fixed Spacing: %d',fixed_spacing_bool);
    %fprintf('Current spacing of the first CAV: %4.2f \n',S(k,3,1)-S(k,4,1));
end
tsim = toc;

fprintf('Variable dimension %6.4f \n', L);
fprintf('Simulation ends at %6.4f seconds \n', tsim);
cost = 0;
for i=1:size(y,2)
    % cost = cost+y(:,i)'*Q*y(:,i)+u(:,i)'*R*u(:,i);
    cost = cost+(y(:,i)-y_star)'*Q*(y(:,i)-y_star)+u(:,i)'*R*u(:,i);
end