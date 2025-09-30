clear;
clc;

load('Experiment2_deepc_y.mat');
load('Experiment2_baseline_y.mat');
load('Experiment2_reference_y.mat');

y_p = yex21(:,200:345);
y_px = y_p(1,:);
y_py = y_p(2,:);
y_pz = y_p(3,:);

y_k = yex22(:,200:345);
y_kx = y_k(1,:);
y_ky = y_k(2,:);
y_kz = y_k(3,:);

tx = Yref_ext(:,1);
ty = Yref_ext(:,2);
tz = Yref_ext(:,3);

figure(1);
plot3(y_px,y_py,y_pz,'-r','LineWidth', 2,'MarkerSize', 1);
hold on;

plot3(y_kx,y_ky,y_kz,'-b','LineWidth', 2,'MarkerSize', 1);
hold on;

plot3(tx,ty,tz,'-k','LineWidth', 2);
set(gcf,'Units','centimeters','Position',[0 0 10 7.5]);  % 图大小 8cm × 6cm

zlim([-100, -60]);
ax = gca; % Get Current Axes
set(ax, ...
    'FontName', 'Times New Roman', ... % 设置坐标轴字体
    'FontWeight', 'bold',...
    'FontSize',10);             % 设置坐标轴字体为粗体
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');

lgd = legend('DeePC','BA','Ref');
set(lgd, ...
    'FontName', 'Times New Roman', ... % 设置 Legend 字体
    'FontWeight', 'bold',...
    'FontSize',10, ...
    'Location','northeast', ...
    'ItemTokenSize', [10, 8]);             % 设置 Legend 字体为粗体

fig = gcf;
fig.PaperPositionMode = 'auto'; % 设置为auto模式，使得PaperPosition会根据屏幕显示的大小来设置
fig_pos = fig.PaperPosition;    % 获取PaperPosition，格式为 [left, bottom, width, height]
fig.PaperSize = [fig_pos(3) fig_pos(4)]; % 将PaperSize的宽高设置为与PaperPosition一致
print(fig, '_figs\experiment2_1.pdf', '-dpdf');

figure(2);
plot3(y_px,y_py,y_pz,'-r','LineWidth', 2,'MarkerSize', 1);
hold on;

plot3(y_kx,y_ky,y_kz,'-b','LineWidth', 2,'MarkerSize', 1);
hold on;

plot3(tx,ty,tz,'-k','LineWidth', 2);
set(gcf,'Units','centimeters','Position',[0 0 10 7.5]);  % 图大小 8cm × 6cm

zlim([-100, -60]);
% xlim([-75, 75]);
% ylim([-75, 75]);
ax = gca; % Get Current Axes
set(ax, ...
    'FontName', 'Times New Roman', ... % 设置坐标轴字体
    'FontWeight', 'bold',...
    'FontSize',10);             % 设置坐标轴字体为粗体
xlabel('x (mm)');
ylabel('y (mm)');
zlabel('z (mm)');

% lgd = legend('DeePC','BA','Ref');
% set(lgd, ...
%     'FontName', 'Times New Roman', ... % 设置 Legend 字体
%     'FontWeight', 'bold',...
%     'FontSize',10, ...
%     'Location','northeast');             % 设置 Legend 字体为粗体

view(0,90);

fig = gcf;
fig.PaperPositionMode = 'auto'; % 设置为auto模式，使得PaperPosition会根据屏幕显示的大小来设置
fig_pos = fig.PaperPosition;    % 获取PaperPosition，格式为 [left, bottom, width, height]
fig.PaperSize = [fig_pos(3) fig_pos(4)]; % 将PaperSize的宽高设置为与PaperPosition一致
print(fig, '_figs\experiment2_2.pdf', '-dpdf');
