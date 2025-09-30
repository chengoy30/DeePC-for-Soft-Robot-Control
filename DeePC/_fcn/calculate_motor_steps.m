function motor_steps = calculate_motor_steps(phi_b_deg, gamma_g_deg)
    % 函数说明: 根据期望的弯曲角度和方向，计算连续体机器人各驱动电机的步数
    % 输入:
    %   phi_b_deg:   骨干的目标弯曲角度 (°)
    %   gamma_g_deg: 弯曲平面的目标方向角度 (°)
    % 输出:
    %   motor_steps: 一个包含n个元素的列向量，每个元素对应一个电机的步数

    % --- 常量定义 ---
    L = 88;              % 机器人骨干的长度 (mm)
    n = 3;               % 缆绳的总数量
    d = 10;              % 缆绳距离中心的径向距离 (mm)
    diameter = 15;       % 驱动轮盘直径 (mm)
    steps_per_rev = 200; % 步进电机每转一圈的步数

    % --- 输入角度转换为弧度 ---
    phi_b_rad = deg2rad(phi_b_deg);
    gamma_g_rad = deg2rad(gamma_g_deg);

    % --- 运动学计算 (向量化) ---
    kappa_b = phi_b_rad / L; % 骨干的曲率

    % 为每根缆绳计算其相对于弯曲平面的角度
    i = (1:n)'; % 创建一个列向量 [1; 2; 3]
    beta_rad = (2*pi*(i-1)/n) - gamma_g_rad;

    % 计算每根缆绳到中性轴的距离
    d_i = d * cos(beta_rad);

    % 计算每根缆绳的理论长度
    l_i = L * (1 - kappa_b .* d_i);

    % 计算每根缆绳的长度变化量 (需要收缩的长度)
    delta_l = L - l_i;
    
    % --- 将长度变化量转换为电机步数 ---
    circumference = diameter * pi; % 轮盘周长
    motor_steps = (delta_l / circumference) * steps_per_rev;

end

