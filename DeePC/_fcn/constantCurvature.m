function [x, y, z] = constantCurvature(theta, phi, L)
% constantCurvature  恒定曲率模型末端位置计算
%
%   [x, y, z] = constantCurvature(theta, phi, L)
%
% 输入：
%   theta — 总弯曲角度 (rad)
%   phi   — 弯曲平面方向 (绕 z 轴旋转角，rad)
%   L     — 软体臂总弧长
%
% 输出：
%   x, y, z — 末端在基座坐标系下的坐标
%
% 公式：
%   kappa = theta / L;
%   x' = (1/kappa)*(1 - cos(theta));
%   z' = (1/kappa)*sin(theta);
%   x = x'*cos(phi);
%   y = x'*sin(phi);
%   z = z';

    % 处理 theta → 0 的极限情况
    if abs(theta) < eps
        % 当 theta 非常小时，臂近似直线，末端在 z 轴方向延伸 L
        x = 0;
        y = 0;
        z = L;
        return;
    end

    % 计算常数曲率模型
    kappa = theta / L;
    x_plane = (1/kappa) * (1 - cos(theta));
    z     = (1/kappa) * sin(theta);

    % 将平面内坐标绕 z 轴旋转到全局坐标系
    x = x_plane * cos(phi);
    y = x_plane * sin(phi);
end
