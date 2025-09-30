clc; clear;
close all
addpath('_fcn');

rng(9);

%% === 初始化 NatNet 客户端 ===
dllPath = 'D:\FILES\NatNetSDK\lib\x64\NatNetML.dll';
NET.addAssembly(dllPath);
client = NatNetML.NatNetClientML();
client.Initialize('127.0.0.1', '127.0.0.1');

% 可选：若你知道刚体ID，填入以更稳；否则留空自动选择第一个 Tracked 刚体
targetRBID = [];  % 例如 targetRBID = 1;

%% === 打开串口（统一用 LF 做包尾）===
portName = "COM5";        % 根据实际改
baudRate = 1000000;       % 与 Arduino 端保持一致
s = serialport(portName, baudRate);
% s.Timeout = 2;                       % 防阻塞
configureTerminator(s, "LF", 43);  

% 可选握手：如果 Arduino 会主动发一行（例如 "ARDUINO_READY"）
try
    line = readline(s);
    fprintf("Arduino says: %s\n", line);
catch
    warning("No handshake line within timeout, continue anyway.");
end

%% === 期望起始位置（毫米）并计算偏移 T ===
desired_start_pos_mm = [0.0; 0.0; -90.0];
initial_T_calculated = false;
T = [0.0; 0.0; 0.0]; % 初始化偏移

% 循环等待直到获取有效刚体
javaPause(0.1);
while ~initial_T_calculated
    initial_frame = client.GetLastFrameOfData();
    if isempty(initial_frame)
        javaPause(0.1);
        continue;
    end

    rb = getTrackedRigidBody(initial_frame, targetRBID);
    if ~isempty(rb)
        initial_pos_m  = [rb.x; rb.y; rb.z];
        initial_pos_mm = initial_pos_m * 1000.0;
        T = desired_start_pos_mm - initial_pos_mm;
        initial_T_calculated = true;
        fprintf('✅ 刚体起始位置校准完成。偏移量 T = [%.2f, %.2f, %.2f] mm\n', T(1), T(2), T(3));
    else
        disp("⚠️ 未检测到被跟踪的刚体，请确保刚体在视野中且被激活。");
        javaPause(0.5);
    end
end

T = [-108.2107; 129.3558; -744.0654];

%% === 变量与数据 ===
target_c = 0;
target_c2 = 0;
target_c3 = 0;

% i_data = 2;
load(['_data\trajectory_data_collection\ioput_v2','.mat']);  % 需要包含 ud, yd (及可能的 ed)

m_ctr = 3;
p_ctr = 3;

% iteration
total_step     = 440;

Ts    = 0.25;          % 采样周期
TsNs  = int64(Ts * 1e9);  % 纳秒，int64 防精度/溢出

% Desired setpoint
u_star = [0;0;0];      % Desired input

L_arm  = 90;           % 软体臂长度 (重命名避免和秩混淆)
theta  = pi/4;         % 弯曲角度 (弧度)

% Performance cost
weight_y = 30;                  % 位置误差权重
R        = 5e-3 * eye(m_ctr);    % 控制输入权重
Q        = weight_y * eye(p_ctr);
lambda_g = 450;                  % ||g||_2^2
lambda_y = 1000;           

% Constraints
u_limit = [-100,100;...
           -100,100;...
           -100,100];
y_limit = [-100,100;...
           -100,100;...
           -100,0];

u         = zeros(m_ctr,total_step);     % control input log
y         = zeros(p_ctr,total_step);     % output log
pr_status = zeros(total_step,1);         % QP status

% Hankel / Page
iMa      = 1;              % 1: Hankel
Tini     = 20;             % past length
N        = 30;             % future length (window)

n_length = 1150;
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
    E_page  = page_matrix(ed,Tini+N);
    Ep = E_page(1:Tini,1:n_length);
    Ef = E_page((Tini+1):end,1:n_length);
    Y_page  = page_matrix(yd,Tini+N);
    Yp = Y_page(1:Tini*p_ctr,1:n_length);
    Yf = Y_page((Tini*p_ctr+1):end,1:n_length);
end

% SVD based dimension reduction
data_length = 600;
AA = [Up;Yp;Uf;Yf];
[U_aa,S_aa,V_aa] = svd(AA);
AA_L = AA*V_aa(:,1:data_length);
% AA_L = U_aa(:,1:L);
% AA_L = AA(:,1:L);
Up = AA_L(1:size(Up,1),:);
Yp = AA_L(size(Up,1)+1:size(Up,1)+size(Yp,1),:);
Uf = AA_L(size(Up,1)+size(Yp,1)+1:size(Up,1)+size(Yp,1)+size(Uf,1),:);
Yf = AA_L(size(Up,1)+size(Yp,1)+size(Uf,1)+1:end,:);

num_points = 800; 
trajectory = zeros(num_points, 3);
for i = 1:num_points
    % 根据步数区间决定 theta, phi
    if i <= 120
        theta = deg2rad(20);
        phi   = deg2rad(0);
    elseif i <= 240
        theta = deg2rad(40);
        phi   = deg2rad(60);
    elseif i <= 360
        theta = deg2rad(60);
        phi   = deg2rad(120);
    elseif i <= 480
        theta = deg2rad(50);
        phi   = deg2rad(180);
    elseif i <= 600
        theta = deg2rad(45);
        phi   = deg2rad(240);
    elseif i <= 800
        theta = deg2rad(35);
        phi   = deg2rad(300);
    end

    % 计算末端点
    [x_r, y_r, z_r] = constantCurvature(theta, phi, L_arm);

    % 保存轨迹（注意你之前用了 -z_r）
    trajectory(i, :) = [x_r, y_r, -z_r];
end

window_size = N;
Yref_ext = trajectory;

%% === 初始化 past 数据 ===
% initial past data in control process
uini = zeros(m_ctr,Tini);
yini = zeros(p_ctr,Tini);
yini(end, :) = -90;

% 更新 u/y 初值
u(:,1:Tini) = uini;
y(:,1:Tini) = yini;

%% === 主控制循环 ===
startTime = java.lang.System.nanoTime();  % 重新起步计时（相对步）

for k = Tini:total_step-1
    % 参考窗口（长度 N）
    y_star_win = Yref_ext(k-Tini+1 : k-Tini+window_size, :);
    y_star = reshape(y_star_win.', [], 1);

    % 解 DeePC QP
    [u_opt,y_opt,pr] = qp_DeePC_v4(Up,Yp,Uf,Yf, uini,yini, Q,R, y_star,u_star, ...
                                lambda_g,lambda_y, u_limit,y_limit);
    
    % 输出命令（第一步）
    target_c = int2str(round(u_opt(1,1)));
    target_c2 = int2str(round(u_opt(2,1)));
    target_c3 = int2str(round(u_opt(3,1)));
    command = target_c + "," + target_c2 + "," + target_c3 + ",";

    % 对齐到本回合的目标时刻（k 从 Tini 开始）
    stepIdx    = int64(k - Tini + 1);
    targetTime = startTime + stepIdx * TsNs;

    currTime = java.lang.System.nanoTime();
    if currTime < targetTime
        remain_ns = double(targetTime - currTime);
        if remain_ns > 2e6
            java.lang.Thread.sleep( max(0, floor((remain_ns - 1.5e6)/1e6)) );
        end
        while java.lang.System.nanoTime() < targetTime
        end
    else
        fprintf("⚠️ Overrun by %.2f ms at k=%d\n", (double(currTime - targetTime)/1e6), k);
    end

    % 发送
    writeline(s, command);

    % 记录与滚动窗口
    u_k = round(u_opt(1:m_ctr,1));
    u(:,k+1)       = u_k;
    pr_status(k) = pr;

    % 读取最新位姿
    frame = client.GetLastFrameOfData();
    if isempty(frame)
        pos_m = [NaN, NaN, NaN];
    else
        rb = getTrackedRigidBody(frame, targetRBID);
        if ~isempty(rb)
            pos_m = [rb.x; rb.y; rb.z];
        else
            pos_m = [NaN; NaN; NaN];
        end
    end
    pos_mm   = pos_m * 1000.0 + T;
    y(:,k+1) = pos_mm;

    % 更新 past 数据窗口
    uini = u(:,k-Tini+2:k+1);
    yini = y(:,k-Tini+2:k+1);

    % 进度
    pct = 100 * (k - Tini + 1) / (total_step - Tini);
    fprintf('process... %5.1f%% | pr=%d\n', pct, pr);
end

%% === 资源清理 ===
c = onCleanup(@()cleanup_all(s, client));

%% ========== 本文件局部函数 ==========
function rb = getTrackedRigidBody(frame, targetID)
    rb = [];
    if isempty(frame) || frame.nRigidBodies == 0
        return;
    end
    % 优先按 ID 查找
    if ~isempty(targetID)
        for i = 0:frame.nRigidBodies-1
            r = frame.RigidBodies.Get(i);
            if r.ID == targetID && r.Tracked
                rb = r; return;
            end
        end
    end
    % 退而求其次：找第一个被 Tracked 的刚体
    for i = 0:frame.nRigidBodies-1
        r = frame.RigidBodies.Get(i);
        if r.Tracked
            rb = r; return;
        end
    end
end

function cleanup_all(s, client)
    try
        if ~isempty(s) && isvalid(s)
            delete(s);
        end
    catch
    end
    try
        clear client;
    catch
    end
    try
        close all;
    catch
    end
    fprintf("✅ Cleaned up resources.\n");
end



save('Experiment1_v2.mat');




