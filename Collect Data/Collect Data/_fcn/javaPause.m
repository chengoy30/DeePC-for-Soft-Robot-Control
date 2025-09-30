function javaPause(dt)
% javaPause 精确暂停 dt 秒 （例如 dt = 0.05）
%
%   javaPause(0.05) 相当于 pause(0.05)，但使用 Java 的高精度时钟。

    % 将秒转纳秒，并计算目标时刻
    targetTime = java.lang.System.nanoTime() + int64(dt * 1e9);

    % 先做一次粗略的睡眠，留最后一小段用忙等
    % 这里取 90% 的时间做系统睡眠，剩余 10% 用忙等
    coarseSleepMs = floor(dt * 1000 * 0.9);
    if coarseSleepMs > 0
        java.lang.Thread.sleep(coarseSleepMs);
    end

    % 忙等直到精确到目标时刻
    while java.lang.System.nanoTime() < targetTime
        % 空循环
    end
end