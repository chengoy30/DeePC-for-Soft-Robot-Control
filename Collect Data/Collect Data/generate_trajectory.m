% 清除工作区和命令窗口
clear; clc; close all;

% 定义总时间范围和采样频率
total_time = 300; % 信号总时长
fs = 4;        % 采样频率 (Hz)，用于生成平滑的曲线
t_fine = 0:1/fs:total_time; % 细粒度的时间向量，用于绘制信号

%% 信号 phi_b
% 定义 phi_b 的关键时间点和对应的幅值
t_phi_b_pts =   [0, 15, 30, 38, 55, 63, 78, 85, 102, 110, 120, 140, 151, 160, 170, 185, 200, 210, 225, 235, 250, 265, 277, 290, 300];
val_phi_b_pts = [0, 40, 40, 55, 55, 45, 45, 35,  35,  10,  10,  47,  47,  33,  33,  62,  62,  42,  42,  15,  15,  75,  75,  28,  28];

% 使用 interp1 进行线性插值
% 'linear' 表示在两个数据点之间使用直线连接（即斜坡）
% 'previous' 可以用于生成阶跃保持，但这里我们希望斜坡和保持，所以线性插值更合适。
phi_b = interp1(t_phi_b_pts, val_phi_b_pts, t_fine, 'linear', 'extrap'); % 'extrap' 用于处理 t_fine 超出 t_u2_pts 范围的情况

% 确保信号不会在超出定义范围后继续斜坡或插值，通常在末端应保持最后一个值
% 或者，如果最后一点是保持，那么 interp1('linear') 就能正确处理。
% 这里我们确保最后一点是保持的
if t_fine(end) > t_phi_b_pts(end)
    phi_b(t_fine > t_phi_b_pts(end)) = val_phi_b_pts(end);
end

%% 信号 gamma_g
% 定义 gamma_g 的关键时间点和对应的幅值
t_gamma_g_pts =   [0,  30,  40,  50,  60,  75,  85, 102, 110, 150, 155, 160, 165, 175, 185, 235, 240, 265, 272, 282, 287, 297, 300];
val_gamma_g_pts = [0, 320, 320, 180, 180, 240, 240,  80,  80, 365, 365, 370, 370, 350, 350,   0,   0, 260, 260, 180, 180, 230, 230]; % 最后一个点与58s相同以确保保持

% 使用 interp1 进行线性插值
gamma_g = interp1(t_gamma_g_pts, val_gamma_g_pts, t_fine, 'linear', 'extrap');

% 同样，确保信号在超出定义范围后保持最后一个值
if t_fine(end) > t_gamma_g_pts(end)
    gamma_g(t_fine > t_gamma_g_pts(end)) = val_gamma_g_pts(end);
end


%% 绘图
figure(1);
plot(t_fine, phi_b, 'r-', 'LineWidth', 2, 'DisplayName', '\phi_b');
title('Ramp-and-Hold Input with Adjustable Hold Duration');
xlabel('Time [s]');
ylabel('Control Input');
grid on;
legend('show');

figure(2);
plot(t_fine, gamma_g, 'g-', 'LineWidth', 2, 'DisplayName', '\gamma_g');
title('Ramp-and-Hold Input with Adjustable Hold Duration');
xlabel('Time [s]');
ylabel('Control Input');
grid on;
legend('show');
ylim([0, 375]); % 根据图示调整Y轴范围，使其包含所有信号
xlim([0, total_time]); % 确保X轴范围正确
set(gca, 'Color', [0.94 0.94 0.94]); % 尝试匹配背景颜色

phi_b_deg_trajectory = phi_b(2:end);
gamma_g_deg_trajectory = gamma_g(2:end);
save('trajectory.mat', 'phi_b_deg_trajectory', 'gamma_g_deg_trajectory');