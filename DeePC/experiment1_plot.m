addpath('_fcn');
load('Experiment1_v1.mat');
new_y = y(:,21:380);

% 假设 new_y 是 3x480，每列 [x;y;z]
num_points = size(new_y, 2);
angles = zeros(2, num_points);   % 存放 [theta; phi]

L = 90;  % 你软体臂的弧长

for i = 1:num_points
    x = new_y(1,i);
    y = new_y(2,i);
    z = new_y(3,i);

    [theta_i, phi_i, ~, ~] = invertCC_fromXYZ(x, y, -z, L);

    angles(:,i) = [theta_i; phi_i];   % 每列存放 (theta, phi)
end

% 如果想要角度制：
angles_deg = rad2deg(angles);

steps = 1:size(angles_deg, 2);   % 步数 1~480

theta_deg = angles_deg(1,:);     % 取第一行：theta
phi_deg   = angles_deg(2,:);     % 取第二行：phi

% Reference theta
theta_ref = zeros(1,360);
theta_ref(1:120)   = 20;   % 前120步
theta_ref(121:240) = 40;   % 121-240步
theta_ref(241:360) = 60;   % 241-360步

% Reference phi
phi_ref = zeros(1,360);
phi_ref(1:120)   = 0;
phi_ref(121:240) = 60;
phi_ref(241:360) = 120;

% 计算误差
theta_err = theta_deg - theta_ref;
phi_err   = phi_deg - phi_ref;

figW = 18.1;   % cm，IEEE 双栏整页宽 ~7.125in ≈ 18.1cm
figH = 11.5;   % cm，自己调到合适的纵横比
fs_tick = 9;   % 坐标刻度字号
fs_label = 10; % 轴标签字号
fs_legend = 9; % 图例字号
lw = 1.5;      % 曲线线宽

% 图1：theta vs 步数
figure(1);
plot(steps, theta_deg, 'r-', 'LineWidth', 1.5); hold on;
plot(steps, theta_ref, 'k--', 'LineWidth', 1.5);
set(gcf,'Units','centimeters','Position',[0 0 8 6]);  % 图大小 8cm × 6cm
xlim([0 360]);
ylim([10 100]);
% xlabel('Step','FontSize',fs_label);
% ylabel('Bending Angle \phi_b (deg)','FontSize',fs_label);
% legend('Exp Results','Ref Results','Location','northeast','FontSize',fs_legend);
% 1. 获取坐标轴句柄，并设置字体和加粗
ax = gca; % Get Current Axes
set(ax, ...
    'FontName', 'Times New Roman', ... % 设置坐标轴字体
    'FontWeight', 'bold');             % 设置坐标轴字体为粗体

% 2. 设置 X 和 Y 轴标签（现在会自动继承上面的字体设置）
xlabel('Step','FontSize',fs_label);
ylabel('Bending Angle \phi_b (deg)','FontSize',fs_label);

% 3. 创建 Legend 并获取其句柄，然后单独设置其属性
lgd = legend('Exp Results','Ref Results','Location','northeast','FontSize',fs_legend);
set(lgd, ...
    'FontName', 'Times New Roman', ... % 设置 Legend 字体
    'FontWeight', 'bold');             % 设置 Legend 字体为粗体
% 2. 设置属性并保存
fig = gcf;
fig.PaperPositionMode = 'auto'; % 设置为auto模式，使得PaperPosition会根据屏幕显示的大小来设置
fig_pos = fig.PaperPosition;    % 获取PaperPosition，格式为 [left, bottom, width, height]
fig.PaperSize = [fig_pos(3) fig_pos(4)]; % 将PaperSize的宽高设置为与PaperPosition一致

% 3. 使用print函数保存
print(fig, '_figs\experiment1_1.pdf', '-dpdf');


% 图2：phi vs 步数
figure(2);
plot(steps, phi_deg, 'r-', 'LineWidth', 1.5); hold on;
plot(steps, phi_ref, 'k--', 'LineWidth', 1.5);
set(gcf,'Units','centimeters','Position',[0 0 8 6]);  % 图大小 8cm × 6cm
xlim([0 360]);
ylim([-10 180]);
ax = gca; % Get Current Axes
set(ax, ...
    'FontName', 'Times New Roman', ... % 设置坐标轴字体
    'FontWeight', 'bold');             % 设置坐标轴字体为粗体
xlabel('Step','FontSize',fs_label);
ylabel('Orientation Angle \gamma_g (deg)','FontSize',fs_label);
lgd = legend('Exp Results','Ref Results','Location','northeast','FontSize',fs_legend);
set(lgd, ...
    'FontName', 'Times New Roman', ... % 设置 Legend 字体
    'FontWeight', 'bold');             % 设置 Legend 字体为粗体
fig = gcf;
fig.PaperPositionMode = 'auto'; % 设置为auto模式，使得PaperPosition会根据屏幕显示的大小来设置
fig_pos = fig.PaperPosition;    % 获取PaperPosition，格式为 [left, bottom, width, height]
fig.PaperSize = [fig_pos(3) fig_pos(4)]; % 将PaperSize的宽高设置为与PaperPosition一致
print(fig, '_figs\experiment1_2.pdf', '-dpdf');

% Theta 误差
figure(3);
plot(steps, theta_err, 'b-', 'LineWidth', 1.5);
set(gcf,'Units','centimeters','Position',[0 0 8 6]);  % 图大小 8cm × 6cm
ax = gca; % Get Current Axes
set(ax, ...
    'FontName', 'Times New Roman', ... % 设置坐标轴字体
    'FontWeight', 'bold');             % 设置坐标轴字体为粗体
xlabel('Step');
ylabel('Error in \phi_b (deg)');
xlim([0 360]);
ylim([-25 25]);
fig = gcf;
fig.PaperPositionMode = 'auto'; % 设置为auto模式，使得PaperPosition会根据屏幕显示的大小来设置
fig_pos = fig.PaperPosition;    % 获取PaperPosition，格式为 [left, bottom, width, height]
fig.PaperSize = [fig_pos(3) fig_pos(4)]; % 将PaperSize的宽高设置为与PaperPosition一致
print(fig, '_figs\experiment1_3.pdf', '-dpdf');

% Phi 误差
figure(4);
plot(steps, phi_err, 'b-', 'LineWidth', 1.5);
set(gcf,'Units','centimeters','Position',[0 0 8 6]);  % 图大小 8cm × 6cm
ax = gca; % Get Current Axes
set(ax, ...
    'FontName', 'Times New Roman', ... % 设置坐标轴字体
    'FontWeight', 'bold');             % 设置坐标轴字体为粗体
xlabel('Step');
ylabel('Error in \gamma_g (deg)');
xlim([0 360]);
ylim([-25 25]);
fig = gcf;
fig.PaperPositionMode = 'auto'; % 设置为auto模式，使得PaperPosition会根据屏幕显示的大小来设置
fig_pos = fig.PaperPosition;    % 获取PaperPosition，格式为 [left, bottom, width, height]
fig.PaperSize = [fig_pos(3) fig_pos(4)]; % 将PaperSize的宽高设置为与PaperPosition一致
print(fig, '_figs\experiment1_4.pdf', '-dpdf');
