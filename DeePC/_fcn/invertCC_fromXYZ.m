function [theta, phi, is_consistent, L_est] = invertCC_fromXYZ(x, y, z, L)
% 从单点坐标反解恒曲率模型的 (theta, phi)，并与弧长 L 做一致性检查
    rho  = hypot(x, y);
    phi  = atan2(y, x);                % 弯曲平面方向
    theta = 2 * atan2(rho, z);         % 总弯曲角度（主值，通常 0~π）

    % 处理直线极限（rho≈0 且 z>0）
    if rho < eps && z > 0
        theta = 0;
        % phi 任意：保留上面的 atan2 结果或设为 0 都可
    end

    % 估计曲率半径 r 与由坐标推得的弧长 L_est
    if abs(sin(theta)) > eps
        r = z / sin(theta);
    else
        % theta≈0 或≈π 的边界用另一式避免除零
        denom = (1 - cos(theta));
        if abs(denom) > eps
            r = rho / denom;
        else
            r = inf; % 近似直线
        end
    end
    L_est = theta * r;

    % 与给定 L 的一致性（允许小相对误差）
    tol = 1e-6 * max(1, abs(L));
    is_consistent = isfinite(L_est) && (abs(L_est - L) <= tol);
end
