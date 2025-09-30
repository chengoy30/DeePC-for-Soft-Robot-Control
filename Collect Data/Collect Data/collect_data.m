clc; clear;
addpath('_fcn');

% === 初始化 NatNet 客户端 ===
dllPath = 'D:\FILES\NatNetSDK\lib\x64\NatNetML.dll';
NET.addAssembly(dllPath);
client = NatNetML.NatNetClientML();
client.Initialize('127.0.0.1', '127.0.0.1');

% 定义你期望的刚体起始点（毫米）
desired_start_pos_mm = [0.0, 0.0, -90.0];

% 尝试获取初始帧以计算偏移量 T
initial_T_calculated = false;
T = [0.0, 0.0, 0.0]; % 初始化 T，以防未能成功获取初始值

% 循环等待直到获取到第一个有效的刚体数据，以计算偏移量 T
javaPause(0.1);
while ~initial_T_calculated
    initial_frame = client.GetLastFrameOfData();
    if isempty(initial_frame)
        javaPause(0.1); % 等待数据
        continue;
    end

    if initial_frame.nRigidBodies > 0
        initial_rigidBody = initial_frame.RigidBodies.Get(0); % 获取第一个刚体

        if ~isempty(initial_rigidBody) && initial_rigidBody.Tracked
            % 获取刚体在初始时刻的真实位置（毫米）
            initial_pos_m = [initial_rigidBody.x, initial_rigidBody.y, initial_rigidBody.z];
            initial_pos_mm = initial_pos_m * 1000.0; 
            
            % 计算偏移量 T：期望的起始位置 - 实际测量到的起始位置
            T = desired_start_pos_mm - initial_pos_mm;
            initial_T_calculated = true;
            fprintf('✅ 刚体起始位置校准完成。偏移量 T = [%.2f, %.2f, %.2f] mm\n', T(1), T(2), T(3));
        else
            disp("⚠️ 初始刚体 (ID 0) 未被跟踪或无效，请确保刚体在视野中。");
            javaPause(0.5); % 等待并重试
        end
    else
        disp("⚠️ 未检测到任何刚体，请确保在 Motive 中创建并激活刚体。");
        javaPause(0.5); % 等待并重试
    end
end

% 1) 打开串口
portName = "COM5";        % 根据实际改成你的串口号
baudRate = 1000000;       % 与 Arduino 端 Serial.begin(...) 保持一致
s = serialport(portName, baudRate);

% 2) 可选：设定终止符（这里用 '+' 作为包尾）
configureTerminator(s, "LF", 43);

% --- 1. 在循环前，准备并打开文件 ---
logFileName = "arduino_log_" + datestr(now, 'yyyy-mm-dd_HH-MM-SS') + ".txt";
% 使用 'a' 模式来追加 (append)，如果文件不存在会自动创建
fileID = fopen(logFileName, 'a'); 

% 这是一个好习惯：检查文件是否成功打开
if fileID == -1
   error('无法打开日志文件进行写入: %s', logFileName);
end

% (高级技巧) 使用 onCleanup 对象可以确保即使程序出错，文件也会被自动关闭
cleanupObj = onCleanup(@() fclose(fileID));

L = 90;   % 机器人骨干的长度，单位: mm
n = 3;     % 缆绳的总数量
d = 10;  % 缆绳距离中心的径向距离, 单位: mm
diameter = 10; % 轮盘直径, 单位: mm
steps_per_rev = 200;    % 步进电机转一圈的步数

t = 0;
target = 0;
target2 = 0;
target3 = 0;
spacer = '\n';

Ts    = 0.25;          % 采样周期，1 秒采一次
TsNs = Ts * 1e9;

% 加载预先生成的轨迹文件
disp("正在加载轨迹文件 trajectory.mat ...");
try
    load('trajectory.mat');
catch
    error('无法加载 trajectory.mat 文件。请先运行 generate_trajectory.m 脚本。');
end

Nstep = size(gamma_g_deg_trajectory, 2);         % 一共要发送多少个样点（可根据需要改）

% 检查轨迹长度是否足够
if length(phi_b_deg_trajectory) < (Nstep - 1) || length(gamma_g_deg_trajectory) < (Nstep - 1)
    error('轨迹文件中的数据点数量少于 Nstep，请重新生成轨迹文件。');
end

% activate/weak the readline command/buffer
readline(s);

% 6) 精确实时循环
startTime = java.lang.System.nanoTime();
for k = 1:Nstep
    phi_b_deg   = phi_b_deg_trajectory(k);
    gamma_g_deg = gamma_g_deg_trajectory(k);
    
    motor_steps = calculate_motor_steps_v2(L, n, d, diameter, steps_per_rev, phi_b_deg, gamma_g_deg);
    
    target = motor_steps(1, 1);
    target2 = motor_steps(2, 1);
    target3 = motor_steps(3, 1);

    % output the target
    target_c = int2str(round(target));
    target_c2 = int2str(round(target2));
    target_c3 = int2str(round(target3));
    command = target_c + "," + target_c2 + "," + target_c3;
    
    % --- 精确定时控制 ---
    % 计算下一次循环应该开始的时间点，并暂停直到那一刻
    targetTime = startTime + k * TsNs;
    currTime = java.lang.System.nanoTime();
    if currTime < targetTime
        % 忙等到目标时刻
        while java.lang.System.nanoTime() < targetTime
            % (注意：这里会占用 CPU，适合短周期高精度场景)
        end
    else
        % 这是性能瓶颈的信号
        disp("警告: 循环执行时间过长，无法维持采样率！");
    end
    
    writeline(s, command);              % write a string to the serial com
    
    javaPause(0.10);
    % === 获取当前 Marker 数据 ===
    frame = client.GetLastFrameOfData();
    if isempty(frame) || frame.nRigidBodies == 0
        pos_m = [NaN, NaN, NaN];
    else
        rigidBody = frame.RigidBodies.Get(0);  
        pos_m = [rigidBody.x, rigidBody.y, rigidBody.z];
    end
    
    % 使用 fprintf 格式化写入，效率非常高
    % %s 代表字符串, \n 代表换行
    pos_mm = pos_m * 1000.0 + T;  % 米 → 毫米转换
    fprintf(fileID, '%s, %s, %s, %f, %f, %f\n', ...
        target_c, target_c2, target_c3, pos_mm(1), pos_mm(2), pos_mm(3) );
    
end

clear s

