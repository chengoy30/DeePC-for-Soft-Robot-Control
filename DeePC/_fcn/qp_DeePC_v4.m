function [u_opt,y_opt,problem_status] = qp_DeePC_v4(Up,Yp,Uf,Yf,...
    uini,yini,Q,R,y_star,u_star,lambda_g,lambda_y,u_limit,y_limit)
% =========================================================================
% DeePC (QP formulation, single decision variable g)
% min   sum_{k=1..N} (y_k - y_ref_k)' Q (y_k - y_ref_k)
%     + sum_{k=1..N} (u_k - u_ref_k)' R (u_k - u_ref_k)
%     + lambda_g * ||g||_2^2
%     + lambda_y * ||Yp*g - yini||_2^2      (软约束；当 lambda_y==0 时改为硬约束 Yp*g = yini)
% s.t. Up*g = uini
%      u_min <= Uf*g <= u_max
%      y_min <= Yf*g <= y_max
% =========================================================================

% 是否有输入/输出约束
if nargin <= 12
    constraint_bool = 0;
else
    constraint_bool = 1;
end

% 尺寸
m     = size(uini,1);          % 输入维度
p     = size(yini,1);          % 输出维度
Tini  = size(Up,1)/m;          % 过去窗口长度
N     = size(Uf,1)/m;          % 未来窗口长度
L     = size(Up,2);            % g 的维度（数据片段个数）

% 向量化数据
uini_col = reshape(uini, [m*Tini,1]);
yini_col = reshape(yini, [p*Tini,1]);
yr_col   = y_star;                                 % 期望输出 (pN x 1)
ur_col   = reshape(u_star*ones(1,N), [m*N,1]);     % 期望输入 (mN x 1)

% 块对角权重
Q_blk = kron(speye(N), Q);      % (pN x pN)
R_blk = kron(speye(N), R);      % (mN x mN)

% ---------------- QP: 0.5*x'Hx + f'x ----------------
% 为了与 quadprog 规范对应，这里构造“带 2 的正规形式”
H = 2*( Yf'*Q_blk*Yf + Uf'*R_blk*Uf + lambda_g*speye(L) );
f = -2*( Yf'*Q_blk*yr_col + Uf'*R_blk*ur_col );

if lambda_y > 0
    H = H + 2*lambda_y*(Yp'*Yp);
    f = f - 2*lambda_y*(Yp'*yini_col);
end

% 对称化 + 轻微 ridge 提升数值稳定性
H = (H + H')/2 + 1e-9*speye(L);

% 线性等式约束
if lambda_y == 0
    % 硬约束 Yp*g = yini
    B = [Up; Yp];
    c = [uini_col; yini_col];
else
    % 软约束，仅保留 Up*g = uini
    B = Up;
    c = uini_col;
end

% 线性不等式约束（盒约束）
if constraint_bool
    umin_col = reshape(u_limit(:,1)*ones(1,N), [m*N,1]);
    umax_col = reshape(u_limit(:,2)*ones(1,N), [m*N,1]);
    ymin_col = reshape(y_limit(:,1)*ones(1,N), [p*N,1]);
    ymax_col = reshape(y_limit(:,2)*ones(1,N), [p*N,1]);

    A = [ Uf;   -Uf;   Yf;   -Yf  ];   %  (2mN+2pN) x L
    b = [ umax_col; -umin_col; ymax_col; -ymin_col ];
else
    A = [];
    b = [];
end

% 求解
options = optimoptions('quadprog', ...
    'Algorithm','interior-point-convex', ...
    'MaxIterations',1e3, ...
    'Display','off');

[g_opt,~,exitflag,~] = quadprog(H, f, A, b, B, c, [], [], [], options);

% 结果
u_opt = Uf * g_opt;
y_opt = Yf * g_opt;
problem_status = exitflag;

end
